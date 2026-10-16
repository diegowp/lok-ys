#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <span>
#include <unordered_map>
#include <optional>
#include <cstdint>

namespace LokYS::Vfs {

struct ZipEntryInfo {
    std::string filename;
    u32 uncompressedSize = 0;
    u32 compressedSize = 0;
    u16 compressionMethod = 0; // 0 = Stored, 8 = Deflated
    u32 localHeaderOffset = 0;
    u32 crc32 = 0;
};

class ZipVfsReader {
public:
    ZipVfsReader() = default;
    ~ZipVfsReader() = default;

    // Load a ZIP archive from an in-memory buffer
    [[nodiscard]] bool LoadFromMemory(std::span<const uint8_t> buffer);

    // Load a ZIP archive from a file on disk into memory
    [[nodiscard]] bool LoadFromFile(const std::string& filePath);

    // Query if a file exists in the archive (case-insensitive search supported)
    [[nodiscard]] bool HasFile(std::string_view filename) const noexcept;

    // Retrieve file metadata
    [[nodiscard]] std::optional<ZipEntryInfo> GetEntryInfo(std::string_view filename) const;

    // Read full file content as bytes (decompresses automatically if Deflated)
    [[nodiscard]] std::optional<std::vector<uint8_t>> ReadFile(std::string_view filename) const;

    // Read file content as string
    [[nodiscard]] std::optional<std::string> ReadText(std::string_view filename) const;

    // List all files in the archive
    [[nodiscard]] std::vector<std::string> ListFiles() const;

    // Reset/clear loaded archive
    void Clear() noexcept;

    // Static helper: Inflate RFC 1951 raw deflate data
    static bool InflateRaw(std::span<const uint8_t> compressed, std::vector<uint8_t>& outDecompressed);

private:
    [[nodiscard]] std::string NormalizePath(std::string_view path) const;

    std::vector<uint8_t> m_ownedData;
    std::span<const uint8_t> m_archiveData;
    std::unordered_map<std::string, ZipEntryInfo> m_entries;
};

} // namespace LokYS::Vfs
