#pragma once

/* ===========================================================================
 * Rebuilding a ProRes clip that was interrupted before its trailer was written.
 *
 * WHAT GOES WRONG. A .mov keeps its sample table -- the moov atom -- at the END
 * of the file, written by av_write_trailer(). Until that runs the file holds
 * every frame of picture and no index, so a power cut, a panic or a crash
 * mid-take costs the WHOLE clip. A frame sequence cannot fail this way: each
 * .gpr is complete the moment it is closed. This is the one thing ProRes gives
 * up by being a movie.
 *
 * WHY IT IS RECOVERABLE AT ALL. ProRes samples are self-delimiting and stored
 * end to end from the start of the mdat payload:
 *
 *     [ 4-byte big-endian frame size ][ 'icpf' ][ ... frame ... ]
 *
 * so walking the mdat reconstructs every frame boundary exactly, with no index
 * and no guessing. The frame header even carries width and height. What it does
 * NOT carry is the frame rate or the profile fourcc, which is what the
 * _recovery.json sidecar written at clip open supplies.
 *
 * HOW THE REPAIR IS DONE. By REMUXING through libavformat rather than by
 * hand-assembling a moov. Writing atoms by hand would avoid copying the data,
 * and it was considered; the reason not to is that a subtly wrong stsd or edts
 * produces a file that opens and plays and is quietly wrong -- exactly the
 * class of failure this whole exercise is about. The muxer that wrote every
 * good clip on the card writes this one too. The cost is one pass of copying,
 * on a path that only runs after something has already gone wrong.
 *
 * The repair is ATOMIC: it writes alongside and renames over, so an interruption
 * during a repair cannot destroy the frames it was trying to save.
 * ========================================================================= */

#include <cstdint>
#include <string>

namespace apertar {
namespace prores_repair {

struct Result {
    bool attempted = false;     /* false when the clip needed nothing */
    bool ok = false;
    uint64_t framesRecovered = 0;
    uint64_t bytesScanned = 0;
    std::string message;
};

/* "<clip>.mov" -> "<clip>_recovery.json", beside the movie so a card carried to
 * another machine brings its own repair information with it. Lives here rather
 * than on the encoder so the repair tool does not have to link the encoder. */
std::string recoverySidecarPath(const std::string &moviePath);

/* True when `path` is a .mov whose moov atom is missing -- i.e. an interrupted
 * take. Cheap: reads the top-level atom headers only. */
bool needsRepair(const std::string &path);

/* Rebuild `path` in place (atomically). A clip that does not need repair is
 * left untouched and reports attempted = false. */
Result repair(const std::string &path);

/* Walk `mediaRoot` for interrupted ProRes takes and repair each one. Returns
 * how many were repaired. Safe to call when the directory does not exist. */
unsigned repairAll(const std::string &mediaRoot);

}  // namespace prores_repair
}  // namespace apertar

