#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace apertar::tiff {

struct MemoryBuffer {
    std::vector<uint8_t> bytes;
};

inline void writePod(MemoryBuffer &buffer, const void *src, size_t len)
{
    const auto *first = static_cast<const uint8_t *>(src);
    buffer.bytes.insert(buffer.bytes.end(), first, first + len);
}

inline void writeUInt16(MemoryBuffer &buffer, uint16_t value)
{
    writePod(buffer, &value, sizeof(value));
}

inline void writeUInt32(MemoryBuffer &buffer, uint32_t value)
{
    writePod(buffer, &value, sizeof(value));
}

inline void patchUInt32(MemoryBuffer &buffer, size_t offset, uint32_t value)
{
    if (offset + sizeof(value) > buffer.bytes.size())
        throw std::runtime_error("TIFF patch offset is outside the buffer.");

    std::memcpy(buffer.bytes.data() + offset, &value, sizeof(value));
}

enum Type : uint16_t {
    Byte = 1,
    Ascii = 2,
    Short = 3,
    Long = 4,
    Rational = 5,
    Undefined = 7,
    SShort = 8,
    SLong = 9,
    SRational = 10,
};

inline constexpr size_t typeSize(Type type)
{
    switch (type) {
    case Byte:
    case Ascii:
    case Undefined:
        return 1;
    case Short:
    case SShort:
        return 2;
    case Long:
    case SLong:
        return 4;
    case Rational:
    case SRational:
        return 8;
    }

    return 0;
}

#pragma pack(push, 1)
struct Entry {
    uint16_t tag = 0;
    uint16_t type = 0;
    uint32_t count = 0;
    uint32_t value = 0;
};
#pragma pack(pop)

class IfdBuilder {
public:
    void add(uint16_t tag, Type type, uint32_t count, const void *payload = nullptr)
    {
        Pending pending;
        pending.tag = tag;
        pending.type = type;
        pending.count = count;

        const size_t len = typeSize(type) * count;
        if (payload && len > 0) {
            const auto *first = static_cast<const uint8_t *>(payload);
            pending.data.assign(first, first + len);
        } else if (len > 0) {
            pending.data.assign(len, 0);
        }

        entries_.push_back(std::move(pending));
    }

    void sort()
    {
        std::sort(entries_.begin(), entries_.end(), [](const Pending &a, const Pending &b) {
            return a.tag < b.tag;
        });
    }

    uint32_t build(MemoryBuffer &buffer)
    {
        return buildAt(buffer, checkedOffset(buffer.bytes.size()));
    }

    uint32_t buildAt(MemoryBuffer &buffer, uint32_t baseOffset)
    {
        sort();

        const uint32_t ifdOffset = checkedOffset(baseOffset + buffer.bytes.size());
        const uint16_t count = static_cast<uint16_t>(entries_.size());
        writeUInt16(buffer, count);

        const size_t dirOffset = buffer.bytes.size();
        buffer.bytes.resize(buffer.bytes.size() + count * sizeof(Entry), 0);
        writeUInt32(buffer, 0);

        std::vector<Entry> finalEntries;
        finalEntries.reserve(entries_.size());

        for (const Pending &pending : entries_) {
            Entry entry;
            entry.tag = pending.tag;
            entry.type = pending.type;
            entry.count = pending.count;

            const size_t len = typeSize(pending.type) * pending.count;
            if (len == 0) {
                entry.value = 0;
            } else if (len <= sizeof(entry.value)) {
                std::memcpy(&entry.value, pending.data.data(), len);
            } else {
                align(buffer, 4);
                entry.value = checkedOffset(baseOffset + buffer.bytes.size());
                writePod(buffer, pending.data.data(), pending.data.size());
                align(buffer, 4);
            }

            finalEntries.push_back(entry);
        }

        std::memcpy(buffer.bytes.data() + dirOffset,
                    finalEntries.data(),
                    finalEntries.size() * sizeof(Entry));

        return ifdOffset;
    }

private:
    struct Pending {
        uint16_t tag = 0;
        Type type = Byte;
        uint32_t count = 0;
        std::vector<uint8_t> data;
    };

    static uint32_t checkedOffset(size_t value)
    {
        if (value > 0xffffffffu)
            throw std::runtime_error("Classic TIFF offset overflow.");
        return static_cast<uint32_t>(value);
    }

    static void align(MemoryBuffer &buffer, size_t alignment)
    {
        const size_t remainder = buffer.bytes.size() % alignment;
        if (remainder)
            buffer.bytes.insert(buffer.bytes.end(), alignment - remainder, 0);
    }

    std::vector<Pending> entries_;
};

} // namespace apertar::tiff
