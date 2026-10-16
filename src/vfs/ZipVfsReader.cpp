#include "LokYS/vfs/ZipVfsReader.hpp"
#include <fstream>
#include <cstring>
#include <algorithm>
#include <cctype>

namespace LokYS::Vfs {

namespace {

// Helper to read 16-bit little endian
inline u16 ReadLE16(const uint8_t* p) {
    return static_cast<u16>(p[0]) | (static_cast<u16>(p[1]) << 8);
}

// Helper to read 32-bit little endian
inline u32 ReadLE32(const uint8_t* p) {
    return static_cast<u32>(p[0]) |
           (static_cast<u32>(p[1]) << 8) |
           (static_cast<u32>(p[2]) << 16) |
           (static_cast<u32>(p[3]) << 24);
}

// Bit reader for RFC 1951 Deflate
class BitReader {
public:
    explicit BitReader(std::span<const uint8_t> data)
        : m_data(data), m_bytePos(0), m_bitBuffer(0), m_bitsCount(0) {}

    u32 ReadBits(u32 count) {
        while (m_bitsCount < count) {
            if (m_bytePos >= m_data.size()) {
                return 0; // EOF
            }
            m_bitBuffer |= (static_cast<u64>(m_data[m_bytePos++]) << m_bitsCount);
            m_bitsCount += 8;
        }
        u32 mask = (1u << count) - 1u;
        u32 result = static_cast<u32>(m_bitBuffer & mask);
        m_bitBuffer >>= count;
        m_bitsCount -= count;
        return result;
    }

    void AlignToByte() noexcept {
        u32 remainder = m_bitsCount % 8;
        if (remainder > 0) {
            m_bitBuffer >>= remainder;
            m_bitsCount -= remainder;
        }
    }

    [[nodiscard]] size_t GetBytePos() const noexcept {
        return m_bytePos - (m_bitsCount / 8);
    }

    [[nodiscard]] bool HasMoreData() const noexcept {
        return m_bytePos < m_data.size() || m_bitsCount > 0;
    }

    bool SkipBytes(size_t count) {
        AlignToByte();
        size_t curPos = GetBytePos();
        if (curPos + count > m_data.size()) {
            return false;
        }
        m_bytePos = curPos + count;
        m_bitBuffer = 0;
        m_bitsCount = 0;
        return true;
    }

    std::span<const uint8_t> GetRemainingSpan() const {
        size_t curPos = GetBytePos();
        if (curPos >= m_data.size()) return {};
        return m_data.subspan(curPos);
    }

private:
    std::span<const uint8_t> m_data;
    size_t m_bytePos;
    u64 m_bitBuffer;
    u32 m_bitsCount;
};

// Canonical Huffman Tree for RFC 1951
struct HuffmanTree {
    static constexpr size_t MaxSymbols = 320;
    u16 counts[16]{};
    u16 symbols[MaxSymbols]{};
    u16 firstCode[16]{};
    u16 firstIndex[16]{};

    bool Build(const u8* lengths, size_t numSymbols) {
        if (numSymbols > MaxSymbols) return false;
        std::memset(counts, 0, sizeof(counts));
        for (size_t i = 0; i < numSymbols; ++i) {
            if (lengths[i] > 15) return false;
            counts[lengths[i]]++;
        }
        counts[0] = 0;

        u16 code = 0;
        u16 symIdx = 0;
        for (int len = 1; len <= 15; ++len) {
            firstCode[len] = code;
            firstIndex[len] = symIdx;
            code = static_cast<u16>(code + counts[len]);
            symIdx = static_cast<u16>(symIdx + counts[len]);
            code = static_cast<u16>(code << 1);
        }

        u16 currOffset[16]{};
        for (int len = 1; len <= 15; ++len) {
            currOffset[len] = firstIndex[len];
        }

        for (size_t i = 0; i < numSymbols; ++i) {
            u8 len = lengths[i];
            if (len > 0) {
                u16 idx = currOffset[len]++;
                if (idx < MaxSymbols) {
                    symbols[idx] = static_cast<u16>(i);
                }
            }
        }
        return true;
    }

    i32 Decode(BitReader& reader) const {
        u16 code = 0;
        for (int len = 1; len <= 15; ++len) {
            code = static_cast<u16>((code << 1) | reader.ReadBits(1));
            u16 count = counts[len];
            if (count > 0) {
                u16 base = firstCode[len];
                if (code >= base && code < base + count) {
                    u16 idx = static_cast<u16>(firstIndex[len] + (code - base));
                    if (idx < MaxSymbols) {
                        return symbols[idx];
                    }
                }
            }
        }
        return -1; // Symbol not found
    }
};

} // namespace

bool ZipVfsReader::InflateRaw(std::span<const uint8_t> compressed, std::vector<uint8_t>& outDecompressed) {
    if (compressed.empty()) {
        return false;
    }

    BitReader reader(compressed);
    bool bFinal = false;

    // RFC 1951 Extra bits lookup tables
    constexpr u16 lengthBase[29] = {
        3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
        35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258
    };
    constexpr u8 lengthExtraBits[29] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
        3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0
    };
    constexpr u16 distBase[30] = {
        1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193,
        257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
    };
    constexpr u8 distExtraBits[30] = {
        0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6,
        7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13
    };
    constexpr u8 clcidx[19] = {
        16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
    };

    // Safety ceiling: prevent allocation attacks or unbounded loops (500 MB max)
    constexpr size_t MaxOutputBytes = 500 * 1024 * 1024;

    while (!bFinal) {
        if (!reader.HasMoreData()) {
            break;
        }

        bFinal = (reader.ReadBits(1) != 0);
        u32 blockType = reader.ReadBits(2);

        if (blockType == 0) {
            // Uncompressed block
            reader.AlignToByte();
            auto rem = reader.GetRemainingSpan();
            if (rem.size() < 4) return false;

            u16 len = ReadLE16(&rem[0]);
            u16 nlen = ReadLE16(&rem[2]);
            if (static_cast<u16>(len ^ 0xFFFF) != nlen) return false;

            if (rem.size() < static_cast<size_t>(4 + len)) return false;
            if (outDecompressed.size() + len > MaxOutputBytes) return false;

            outDecompressed.insert(outDecompressed.end(), rem.data() + 4, rem.data() + 4 + len);
            if (!reader.SkipBytes(4 + len)) return false;
        }
        else if (blockType == 1 || blockType == 2) {
            HuffmanTree litTree;
            HuffmanTree distTree;

            if (blockType == 1) {
                // Fixed Huffman codes
                u8 fixedLitLengths[288];
                for (int i = 0; i <= 143; ++i) fixedLitLengths[i] = 8;
                for (int i = 144; i <= 255; ++i) fixedLitLengths[i] = 9;
                for (int i = 256; i <= 279; ++i) fixedLitLengths[i] = 7;
                for (int i = 280; i <= 287; ++i) fixedLitLengths[i] = 8;
                if (!litTree.Build(fixedLitLengths, 288)) return false;

                u8 fixedDistLengths[32];
                for (int i = 0; i < 32; ++i) fixedDistLengths[i] = 5;
                if (!distTree.Build(fixedDistLengths, 32)) return false;
            }
            else {
                // Dynamic Huffman codes
                u32 hlit = reader.ReadBits(5) + 257;
                u32 hdist = reader.ReadBits(5) + 1;
                u32 hclen = reader.ReadBits(4) + 4;

                if (hlit > 286 || hdist > 32) return false;

                u8 clLengths[19]{};
                for (u32 i = 0; i < hclen; ++i) {
                    clLengths[clcidx[i]] = static_cast<u8>(reader.ReadBits(3));
                }
                HuffmanTree clTree;
                if (!clTree.Build(clLengths, 19)) return false;

                std::vector<u8> litAndDistLengths;
                litAndDistLengths.reserve(hlit + hdist);
                while (litAndDistLengths.size() < hlit + hdist) {
                    i32 sym = clTree.Decode(reader);
                    if (sym < 0 || sym > 18) return false;
                    if (sym < 16) {
                        litAndDistLengths.push_back(static_cast<u8>(sym));
                    } else if (sym == 16) {
                        if (litAndDistLengths.empty()) return false;
                        u8 prev = litAndDistLengths.back();
                        u32 repeat = reader.ReadBits(2) + 3;
                        for (u32 r = 0; r < repeat && litAndDistLengths.size() < hlit + hdist; ++r) {
                            litAndDistLengths.push_back(prev);
                        }
                    } else if (sym == 17) {
                        u32 repeat = reader.ReadBits(3) + 3;
                        for (u32 r = 0; r < repeat && litAndDistLengths.size() < hlit + hdist; ++r) {
                            litAndDistLengths.push_back(0);
                        }
                    } else if (sym == 18) {
                        u32 repeat = reader.ReadBits(7) + 11;
                        for (u32 r = 0; r < repeat && litAndDistLengths.size() < hlit + hdist; ++r) {
                            litAndDistLengths.push_back(0);
                        }
                    }
                }

                if (!litTree.Build(litAndDistLengths.data(), hlit)) return false;
                if (!distTree.Build(litAndDistLengths.data() + hlit, hdist)) return false;
            }

            // Decode Huffman stream
            while (true) {
                i32 sym = litTree.Decode(reader);
                if (sym < 0 || sym > 285) return false;
                if (sym < 256) {
                    if (outDecompressed.size() >= MaxOutputBytes) return false;
                    outDecompressed.push_back(static_cast<uint8_t>(sym));
                } else if (sym == 256) {
                    break; // End of block
                } else {
                    u32 lenIdx = static_cast<u32>(sym - 257);
                    if (lenIdx >= 29) return false;
                    u32 length = lengthBase[lenIdx] + reader.ReadBits(lengthExtraBits[lenIdx]);

                    i32 distSym = distTree.Decode(reader);
                    if (distSym < 0 || distSym >= 30) return false;
                    u32 distance = distBase[distSym] + reader.ReadBits(distExtraBits[distSym]);

                    if (distance == 0 || distance > outDecompressed.size()) return false;
                    if (outDecompressed.size() + length > MaxOutputBytes) return false;

                    size_t srcPos = outDecompressed.size() - distance;
                    for (u32 k = 0; k < length; ++k) {
                        outDecompressed.push_back(outDecompressed[srcPos + k]);
                    }
                }
            }
        }
        else {
            return false; // Reserved / invalid block
        }
    }

    return true;
}

std::string ZipVfsReader::NormalizePath(std::string_view path) const {
    std::string result;
    result.reserve(path.size());
    for (char c : path) {
        if (c == '\\') {
            result.push_back('/');
        } else {
            result.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    // Remove leading "./"
    if (result.rfind("./", 0) == 0) {
        result.erase(0, 2);
    }
    return result;
}

void ZipVfsReader::Clear() noexcept {
    m_entries.clear();
    m_archiveData = {};
    m_ownedData.clear();
}

bool ZipVfsReader::LoadFromFile(const std::string& filePath) {
    Clear();
    std::ifstream file(filePath, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }

    auto fileSize = file.tellg();
    if (fileSize <= 0 || fileSize < 22) { // 22 bytes is minimum EOCD size
        return false;
    }

    m_ownedData.resize(static_cast<size_t>(fileSize));
    file.seekg(0, std::ios::beg);
    if (!file.read(reinterpret_cast<char*>(m_ownedData.data()), fileSize)) {
        Clear();
        return false;
    }

    return LoadFromMemory(m_ownedData);
}

bool ZipVfsReader::LoadFromMemory(std::span<const uint8_t> buffer) {
    if (m_ownedData.empty()) {
        m_archiveData = buffer;
    } else {
        m_archiveData = m_ownedData;
    }

    if (m_archiveData.size() < 22) {
        return false;
    }

    // Search for End of Central Directory Record (EOCD signature 0x06054b50)
    // Starting from the end of the file backwards
    size_t searchLen = std::min<size_t>(m_archiveData.size(), 65535 + 22);
    size_t startPos = m_archiveData.size() - searchLen;
    size_t eocdOffset = 0;
    bool foundEocd = false;

    for (size_t i = m_archiveData.size() - 22; i >= startPos; --i) {
        if (ReadLE32(&m_archiveData[i]) == 0x06054b50) {
            eocdOffset = i;
            foundEocd = true;
            break;
        }
        if (i == 0) break;
    }

    if (!foundEocd) {
        return false;
    }

    const uint8_t* eocd = &m_archiveData[eocdOffset];
    u16 totalEntries = ReadLE16(eocd + 10);
    u32 centralDirSize = ReadLE32(eocd + 12);
    u32 centralDirOffset = ReadLE32(eocd + 16);

    if (centralDirOffset + centralDirSize > m_archiveData.size()) {
        return false;
    }

    // Parse Central Directory
    size_t cdPos = centralDirOffset;
    for (u16 entryIdx = 0; entryIdx < totalEntries; ++entryIdx) {
        if (cdPos + 46 > m_archiveData.size()) {
            break;
        }

        if (ReadLE32(&m_archiveData[cdPos]) != 0x02014b50) {
            break; // Bad central directory signature
        }

        const uint8_t* cd = &m_archiveData[cdPos];
        u16 compressionMethod = ReadLE16(cd + 10);
        u32 crc = ReadLE32(cd + 16);
        u32 compSize = ReadLE32(cd + 20);
        u32 uncompSize = ReadLE32(cd + 24);
        u16 fileNameLen = ReadLE16(cd + 28);
        u16 extraLen = ReadLE16(cd + 30);
        u16 commentLen = ReadLE16(cd + 32);
        u32 localHeaderOffset = ReadLE32(cd + 42);

        cdPos += 46;
        if (cdPos + fileNameLen > m_archiveData.size()) {
            break;
        }

        std::string filename(reinterpret_cast<const char*>(&m_archiveData[cdPos]), fileNameLen);
        cdPos += fileNameLen + extraLen + commentLen;

        // Skip directory entries
        if (filename.empty() || filename.back() == '/' || filename.back() == '\\') {
            continue;
        }

        ZipEntryInfo info;
        info.filename = filename;
        info.compressionMethod = compressionMethod;
        info.compressedSize = compSize;
        info.uncompressedSize = uncompSize;
        info.localHeaderOffset = localHeaderOffset;
        info.crc32 = crc;

        m_entries[NormalizePath(filename)] = info;
    }

    return !m_entries.empty();
}

bool ZipVfsReader::HasFile(std::string_view filename) const noexcept {
    return m_entries.find(NormalizePath(filename)) != m_entries.end();
}

std::optional<ZipEntryInfo> ZipVfsReader::GetEntryInfo(std::string_view filename) const {
    auto it = m_entries.find(NormalizePath(filename));
    if (it != m_entries.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::optional<std::vector<uint8_t>> ZipVfsReader::ReadFile(std::string_view filename) const {
    auto it = m_entries.find(NormalizePath(filename));
    if (it == m_entries.end()) {
        return std::nullopt;
    }

    const auto& info = it->second;
    if (info.localHeaderOffset + 30 > m_archiveData.size()) {
        return std::nullopt;
    }

    const uint8_t* loc = &m_archiveData[info.localHeaderOffset];
    if (ReadLE32(loc) != 0x04034b50) {
        return std::nullopt; // Bad local file header signature
    }

    u16 locFileNameLen = ReadLE16(loc + 26);
    u16 locExtraLen = ReadLE16(loc + 28);

    size_t dataOffset = info.localHeaderOffset + 30 + locFileNameLen + locExtraLen;
    if (dataOffset + info.compressedSize > m_archiveData.size()) {
        return std::nullopt;
    }

    std::span<const uint8_t> compressedData(&m_archiveData[dataOffset], info.compressedSize);

    if (info.compressionMethod == 0) {
        // Stored (no compression)
        std::vector<uint8_t> result(compressedData.begin(), compressedData.end());
        return result;
    } else if (info.compressionMethod == 8) {
        // Deflated
        std::vector<uint8_t> decompressed;
        decompressed.reserve(info.uncompressedSize);
        if (InflateRaw(compressedData, decompressed) && decompressed.size() == info.uncompressedSize) {
            return decompressed;
        }
        return std::nullopt;
    }

    return std::nullopt; // Unsupported compression method
}

std::optional<std::string> ZipVfsReader::ReadText(std::string_view filename) const {
    auto bytes = ReadFile(filename);
    if (!bytes) {
        return std::nullopt;
    }
    return std::string(reinterpret_cast<const char*>(bytes->data()), bytes->size());
}

std::vector<std::string> ZipVfsReader::ListFiles() const {
    std::vector<std::string> result;
    result.reserve(m_entries.size());
    for (const auto& [name, info] : m_entries) {
        result.push_back(info.filename);
    }
    return result;
}

} // namespace LokYS::Vfs
