#include "LokYS/vfs/ZipWriter.hpp"
#include <fstream>
#include <cstring>
#include <algorithm>

namespace LokYS::Vfs {

namespace {

inline void AppendLE16(std::vector<uint8_t>& buf, u16 value) {
    buf.push_back(static_cast<uint8_t>(value & 0xFF));
    buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

inline void AppendLE32(std::vector<uint8_t>& buf, u32 value) {
    buf.push_back(static_cast<uint8_t>(value & 0xFF));
    buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    buf.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    buf.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

} // namespace

void ZipWriter::AddFile(const std::string& filename, std::span<const uint8_t> data) {
    RemoveFile(filename);
    ZipWriteEntry entry;
    entry.filename = filename;
    entry.data.assign(data.begin(), data.end());
    m_entries.push_back(std::move(entry));
}

void ZipWriter::AddText(const std::string& filename, std::string_view text) {
    std::span<const uint8_t> bytes(reinterpret_cast<const uint8_t*>(text.data()), text.size());
    AddFile(filename, bytes);
}

bool ZipWriter::HasFile(std::string_view filename) const noexcept {
    for (const auto& entry : m_entries) {
        if (entry.filename == filename) {
            return true;
        }
    }
    return false;
}

bool ZipWriter::RemoveFile(std::string_view filename) {
    auto it = std::remove_if(m_entries.begin(), m_entries.end(),
        [filename](const ZipWriteEntry& e) { return e.filename == filename; });
    if (it != m_entries.end()) {
        m_entries.erase(it, m_entries.end());
        return true;
    }
    return false;
}

u32 ZipWriter::ComputeCRC32(std::span<const uint8_t> data) noexcept {
    u32 crc = 0xFFFFFFFFu;
    for (uint8_t b : data) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) {
            crc = (crc >> 1) ^ (0xEDB88320u & (-(static_cast<int32_t>(crc & 1u))));
        }
    }
    return ~crc;
}

std::vector<uint8_t> ZipWriter::FinalizeToMemory() const {
    std::vector<uint8_t> buffer;
    buffer.reserve(4096);

    struct EntryMeta {
        u32 localHeaderOffset = 0;
        u32 crc32 = 0;
        u32 size = 0;
    };

    std::vector<EntryMeta> metas;
    metas.reserve(m_entries.size());

    // 1. Write Local File Headers + Payloads
    for (const auto& entry : m_entries) {
        EntryMeta meta;
        meta.localHeaderOffset = static_cast<u32>(buffer.size());
        meta.size = static_cast<u32>(entry.data.size());
        meta.crc32 = ComputeCRC32(entry.data);
        metas.push_back(meta);

        // Local file header signature: 0x04034b50
        AppendLE32(buffer, 0x04034b50u);
        AppendLE16(buffer, 20); // Version needed to extract (2.0)
        AppendLE16(buffer, 0);  // General purpose bit flag
        AppendLE16(buffer, 0);  // Compression method: 0 = Stored
        AppendLE16(buffer, 0);  // Last mod file time
        AppendLE16(buffer, 0);  // Last mod file date
        AppendLE32(buffer, meta.crc32);
        AppendLE32(buffer, meta.size); // Compressed size
        AppendLE32(buffer, meta.size); // Uncompressed size
        AppendLE16(buffer, static_cast<u16>(entry.filename.size()));
        AppendLE16(buffer, 0);  // Extra field length

        // Filename
        buffer.insert(buffer.end(), entry.filename.begin(), entry.filename.end());

        // Uncompressed payload
        buffer.insert(buffer.end(), entry.data.begin(), entry.data.end());
    }

    // 2. Write Central Directory
    u32 cdOffset = static_cast<u32>(buffer.size());

    for (size_t i = 0; i < m_entries.size(); ++i) {
        const auto& entry = m_entries[i];
        const auto& meta = metas[i];

        // Central file header signature: 0x02014b50
        AppendLE32(buffer, 0x02014b50u);
        AppendLE16(buffer, 20); // Version made by
        AppendLE16(buffer, 20); // Version needed to extract
        AppendLE16(buffer, 0);  // General purpose bit flag
        AppendLE16(buffer, 0);  // Compression method: 0 = Stored
        AppendLE16(buffer, 0);  // Last mod file time
        AppendLE16(buffer, 0);  // Last mod file date
        AppendLE32(buffer, meta.crc32);
        AppendLE32(buffer, meta.size); // Compressed size
        AppendLE32(buffer, meta.size); // Uncompressed size
        AppendLE16(buffer, static_cast<u16>(entry.filename.size()));
        AppendLE16(buffer, 0);  // Extra field length
        AppendLE16(buffer, 0);  // File comment length
        AppendLE16(buffer, 0);  // Disk number start
        AppendLE16(buffer, 0);  // Internal file attributes
        AppendLE32(buffer, 0);  // External file attributes
        AppendLE32(buffer, meta.localHeaderOffset);

        // Filename
        buffer.insert(buffer.end(), entry.filename.begin(), entry.filename.end());
    }

    u32 cdSize = static_cast<u32>(buffer.size()) - cdOffset;

    // 3. Write End of Central Directory Record (EOCD)
    // End of central dir signature: 0x06054b50
    AppendLE32(buffer, 0x06054b50u);
    AppendLE16(buffer, 0); // Number of this disk
    AppendLE16(buffer, 0); // Disk with start of central directory
    AppendLE16(buffer, static_cast<u16>(m_entries.size())); // Total entries on this disk
    AppendLE16(buffer, static_cast<u16>(m_entries.size())); // Total entries
    AppendLE32(buffer, cdSize);   // Size of central directory
    AppendLE32(buffer, cdOffset); // Offset of start of central directory
    AppendLE16(buffer, 0); // ZIP file comment length

    return buffer;
}

bool ZipWriter::FinalizeToFile(const std::string& filePath) const {
    auto data = FinalizeToMemory();
    std::ofstream out(filePath, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }
    out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
    return out.good();
}

} // namespace LokYS::Vfs
