#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <string_view>
#include <vector>
#include <span>
#include <cstdint>

namespace LokYS::Vfs {

struct ZipWriteEntry {
    std::string filename;
    std::vector<uint8_t> data;
};

class ZipWriter {
public:
    ZipWriter() = default;
    ~ZipWriter() = default;

    // Add raw byte buffer as a file entry
    void AddFile(const std::string& filename, std::span<const uint8_t> data);

    // Add string content as a file entry
    void AddText(const std::string& filename, std::string_view text);

    // Number of entries currently queued
    [[nodiscard]] size_t GetEntryCount() const noexcept { return m_entries.size(); }

    // Check if entry exists in pending queue
    [[nodiscard]] bool HasFile(std::string_view filename) const noexcept;

    // Remove entry by filename if exists
    bool RemoveFile(std::string_view filename);

    // Clear all pending entries
    void Clear() noexcept { m_entries.clear(); }

    // Finalize and build standard PKZIP archive in memory
    [[nodiscard]] std::vector<uint8_t> FinalizeToMemory() const;

    // Finalize and write standard PKZIP archive directly to disk
    [[nodiscard]] bool FinalizeToFile(const std::string& filePath) const;

    // Static helper: Compute standard IEEE 802.3 32-bit CRC
    static u32 ComputeCRC32(std::span<const uint8_t> data) noexcept;

private:
    std::vector<ZipWriteEntry> m_entries;
};

} // namespace LokYS::Vfs
