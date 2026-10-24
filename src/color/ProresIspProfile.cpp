#include "color/ProresIspProfile.hpp"
#include "util/SettingsFile.hpp"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <yaml.h>

namespace apertar {
namespace {
using Matrix = ProresColorMatrix;
Matrix multiply(const Matrix &a, const Matrix &b)
{
    Matrix c{};
    for (unsigned y = 0; y < 3; ++y)
        for (unsigned x = 0; x < 3; ++x)
            for (unsigned k = 0; k < 3; ++k) c[y * 3 + x] += a[y * 3 + k] * b[k * 3 + x];
    return c;
}
Matrix inverse(const Matrix &m)
{
    Matrix r{m[4]*m[8]-m[5]*m[7], m[2]*m[7]-m[1]*m[8], m[1]*m[5]-m[2]*m[4],
             m[5]*m[6]-m[3]*m[8], m[0]*m[8]-m[2]*m[6], m[2]*m[3]-m[0]*m[5],
             m[3]*m[7]-m[4]*m[6], m[1]*m[6]-m[0]*m[7], m[0]*m[4]-m[1]*m[3]};
    const double det = m[0]*r[0] + m[1]*r[3] + m[2]*r[6];
    if (std::abs(det) < 1e-12) throw std::runtime_error("Singular ProRes gamut matrix");
    for (auto &v : r) v /= det;
    return r;
}
Matrix primaries(double rx, double ry, double gx, double gy, double bx, double by,
                 double wx = .3127, double wy = .3290)
{
    Matrix m{rx/ry, gx/gy, bx/by, 1, 1, 1, (1-rx-ry)/ry, (1-gx-gy)/gy, (1-bx-by)/by};
    const auto inv = inverse(m);
    const double white[3]{wx/wy, 1, (1-wx-wy)/wy};
    for (unsigned x=0; x<3; ++x) {
        double scale=0;
        for (unsigned k=0; k<3; ++k) scale += inv[x*3+k]*white[k];
        for (unsigned y=0; y<3; ++y) m[y*3+x] *= scale;
    }
    return m;
}
Matrix d65ToD60()
{
    const Matrix bradford{.8951,.2664,-.1614, -.7502,1.7135,.0367, .0389,-.0685,1.0296};
    const double from[3]{.3127/.3290, 1, (1-.3127-.3290)/.3290};
    const double to[3]{.32168/.33767, 1, (1-.32168-.33767)/.33767};
    Matrix scale{};
    for (unsigned r=0;r<3;++r) {
        double a=0,b=0;
        for (unsigned c=0;c<3;++c) { a+=bradford[r*3+c]*from[c]; b+=bradford[r*3+c]*to[c]; }
        scale[r*3+r]=b/a;
    }
    return multiply(multiply(inverse(bradford),scale),bradford);
}
struct Document {
    yaml_document_t doc{};
    bool loaded = false;
    ~Document() { if (loaded) yaml_document_delete(&doc); }
};
std::string scalar(yaml_node_t *n)
{
    return n && n->type == YAML_SCALAR_NODE
        ? std::string(reinterpret_cast<char *>(n->data.scalar.value), n->data.scalar.length) : "";
}
yaml_node_t *member(yaml_document_t &d, yaml_node_t *n, const char *name)
{
    if (n && n->type == YAML_MAPPING_NODE)
        for (auto p=n->data.mapping.pairs.start; p<n->data.mapping.pairs.top; ++p)
            if (scalar(yaml_document_get_node(&d,p->key)) == name) return yaml_document_get_node(&d,p->value);
    return nullptr;
}
yaml_node_t *algorithm(yaml_document_t &d, const char *name)
{
    auto root=yaml_document_get_root_node(&d);
    if (auto n=member(d,root,name)) return n;
    auto algorithms=member(d,root,"algorithms");
    if (algorithms && algorithms->type==YAML_SEQUENCE_NODE)
        for (auto p=algorithms->data.sequence.items.start; p<algorithms->data.sequence.items.top; ++p)
            if (auto n=member(d,yaml_document_get_node(&d,*p),name)) return n;
    return nullptr;
}
struct Edit { size_t start, end; std::string text; };
void replace(std::vector<Edit> &edits, yaml_node_t *node, const std::string &text)
{
    if (!node) throw std::runtime_error("Missing ProRes tuning field");
    edits.push_back({node->start_mark.index, node->end_mark.index, text});
}
std::string array(const Matrix &m)
{
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(12) << '[';
    for (unsigned i=0; i<9; ++i) out << (i ? "," : "") << m[i];
    return out.str()+']';
}
}
Matrix proresGamutMatrix(ProresColorProfile profile)
{
    const auto rec709=primaries(.64,.33,.30,.60,.15,.06);
    switch (profile) {
    case ProresColorProfile::ArriLogC4: return multiply(inverse(primaries(.7347,.2653,.1424,.8576,.0991,-.0308)),rec709);
    case ProresColorProfile::SonySLog3: return multiply(inverse(primaries(.766,.275,.225,.800,.089,-.087)),rec709);
    case ProresColorProfile::DavinciIntermediate: return multiply(inverse(primaries(.800,.313,.1682,.9877,.079,-.1155)),rec709);
    case ProresColorProfile::P3D60Linear:
        // P3 primaries with ACES D60 white, not DCI white or Display P3/D65.
        return multiply(inverse(primaries(.680,.320,.265,.690,.150,.060,.32168,.33767)),
                        multiply(d65ToD60(),rec709));
    default: return {1,0,0,0,1,0,0,0,1};
    }
}
bool makeProresIspTuning(const std::string &source, ProresColorProfile profile,
                         bool monochrome, std::string &destination, std::string &error,
                         PreviewExposureReference *reference)
{
    if (profile==ProresColorProfile::Standard) { destination=source; return true; }
    try {
        std::ifstream input(source, std::ios::binary);
        if (!input) throw std::runtime_error("Log ProRes requires a readable sensor tuning file");
        std::string text{std::istreambuf_iterator<char>(input),{}};
        // Keep the source JSON intact except for rendering-related fields.
        if (text.empty() || text.find_first_not_of(" \r\n\t") == std::string::npos ||
            text[text.find_first_not_of(" \r\n\t")]!='{' || text.size()>4*1024*1024)
            throw std::runtime_error("Invalid ProRes source tuning JSON");
        yaml_parser_t parser;
        if (!yaml_parser_initialize(&parser)) throw std::runtime_error("Cannot parse ProRes tuning");
        yaml_parser_set_input_string(&parser,reinterpret_cast<const unsigned char *>(text.data()),text.size());
        Document document;
        document.loaded=yaml_parser_load(&parser,&document.doc);
        yaml_parser_delete(&parser);
        if (!document.loaded) throw std::runtime_error("Invalid ProRes source tuning JSON");
        auto &d=document.doc;
        if (reference) {
            auto gamma=member(d,algorithm(d,"rpi.contrast"),"gamma_curve");
            std::vector<double> values;
            if (gamma && gamma->type==YAML_SEQUENCE_NODE)
                for (auto p=gamma->data.sequence.items.start;p<gamma->data.sequence.items.top;++p) {
                    const auto value=scalar(yaml_document_get_node(&d,*p));
                    size_t used=0; const double n=std::stod(value,&used);
                    if (used!=value.size() || !std::isfinite(n)) throw std::runtime_error("Invalid exposure reference gamma");
                    values.push_back(n);
                }
            if (values.size()<4 || values.size()%2) throw std::runtime_error("Missing exposure reference gamma");
            for (size_t i=0;i<values.size();i+=2)
                if ((i && values[i]<=values[i-2]) || values[i+1]<0 || values[i+1]>65535)
                    throw std::runtime_error("Invalid exposure reference gamma");
            reference->decodeCurve.clear(); reference->displayCurve.clear();
            reference->toRec709=inverse(proresGamutMatrix(profile));
            int logY=0, displayY=0;
            for (unsigned i=0;i<=64;++i) {
                const unsigned x=i<32 ? i*512 : i<48 ? (i-32)*1024+16384 : (i-48)*2048+32768;
                const double logValue=std::clamp(proresLogEncode(profile,x/65535.0),0.0,1.0)*65535;
                size_t k=0;
                while (k+4<values.size() && values[k+2]<x) k+=2;
                const double t=(x-values[k])/(values[k+2]-values[k]);
                const double displayValue=values[k+1]+t*(values[k+3]-values[k+1]);
                // Match PiSP's integer knot values and 14-bit slope limit.
                logY=i ? logY+std::clamp(int(logValue)-logY,0,16383) : int(logValue);
                displayY=i ? displayY+std::clamp(int(displayValue)-displayY,0,16383) : int(displayValue);
                reference->decodeCurve.push_back({logY/65535.0,x/65535.0});
                reference->displayCurve.push_back({x/65535.0,displayY/65535.0});
            }
        }
        std::vector<Edit> edits;
        auto ccm=algorithm(d,"rpi.ccm");
        auto ccms=member(d,ccm,"ccms");
        unsigned count=0;
        if (ccms && ccms->type==YAML_SEQUENCE_NODE) {
            const auto gamut=proresGamutMatrix(profile);
            for (auto p=ccms->data.sequence.items.start; p<ccms->data.sequence.items.top; ++p) {
                auto n=member(d,yaml_document_get_node(&d,*p),"ccm");
                if (!n || n->type!=YAML_SEQUENCE_NODE || n->data.sequence.items.top-n->data.sequence.items.start!=9)
                    throw std::runtime_error("Invalid sensor CCM for log ProRes");
                Matrix matrix{};
                for (unsigned i=0;i<9;++i) {
                    const auto value=scalar(yaml_document_get_node(&d,n->data.sequence.items.start[i]));
                    size_t used=0;
                    matrix[i]=std::stod(value,&used);
                    if (used!=value.size() || !std::isfinite(matrix[i])) throw std::runtime_error("Invalid sensor CCM");
                }
                matrix=multiply(gamut,matrix);
                for (double v:matrix) if (v < -8 || v > 7.999) throw std::runtime_error("ProRes CCM exceeds ISP coefficient range");
                replace(edits,n,array(matrix));
                ++count;
            }
            if (auto saturation=member(d,ccm,"saturation")) replace(edits,saturation,"[0,1,1000000,1]");
        }
        if (!count && !monochrome) throw std::runtime_error("Selected tuning has no calibrated CCM for log ProRes");
        std::ostringstream contrast;
        contrast.imbue(std::locale::classic());
        contrast << std::setprecision(12) << "{\"ce_enable\":0,\"gamma_curve\":[";
        // Include every PiSP LUT knot, below libcamera's 1000-item list limit.
        // PiSP subsequently samples its 64 hardware segments. No per-frame work.
        for (unsigned x=0; x<=65536; x+=256) {
            const unsigned inputCode=std::min(x,65535u);
            contrast << (x ? "," : "") << inputCode << ','
                     << std::clamp(proresLogEncode(profile,inputCode/65535.0),0.0,1.0)*65535;
        }
        contrast << "]}";
        replace(edits,algorithm(d,"rpi.contrast"),contrast.str());
        // Tone mapping is display rendering, not part of any scene-log profile.
        if (auto hdr=algorithm(d,"rpi.hdr"); hdr && hdr->type==YAML_MAPPING_NODE)
            for (auto p=hdr->data.mapping.pairs.start; p<hdr->data.mapping.pairs.top; ++p)
                if (auto enabled=member(d,yaml_document_get_node(&d,p->value),"tonemap_enable")) replace(edits,enabled,"0");
        // libyaml marks count Unicode characters; string::replace uses bytes.
        const auto byteIndex = [&](size_t characters) {
            size_t byte=0;
            for (size_t i=0;i<characters && byte<text.size();++i) {
                ++byte;
                while (byte<text.size() && (static_cast<unsigned char>(text[byte])&0xc0)==0x80) ++byte;
            }
            return byte;
        };
        for (auto &edit:edits) { edit.start=byteIndex(edit.start); edit.end=byteIndex(edit.end); }
        std::sort(edits.begin(),edits.end(),[](const Edit &a,const Edit &b){return a.start>b.start;});
        size_t end=text.size();
        for (const auto &edit:edits) {
            if (edit.start>edit.end || edit.end>end) throw std::runtime_error("Overlapping ProRes tuning fields");
            text.replace(edit.start,edit.end-edit.start,edit.text);
            end=edit.start;
        }
        const char *home=std::getenv("HOME");
        if (!home || !*home) throw std::runtime_error("No HOME for internal ProRes tuning");
        const auto path=std::filesystem::path(home)/".cache/apertar-prores"/(std::string(toString(profile))+".json");
        std::ifstream cached(path,std::ios::binary);
        const std::string previous{std::istreambuf_iterator<char>(cached),{}};
        if (previous!=text && !settingsFile::atomicWrite(path,text)) throw std::runtime_error("Cannot write internal ProRes tuning");
        destination=path.string();
        return true;
    } catch (const std::exception &e) { error=e.what(); return false; }
}
}
