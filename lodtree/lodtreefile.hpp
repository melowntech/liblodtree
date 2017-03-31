#ifndef vts_tools_lodtreefile_hpp_included
#define vts_tools_lodtreefile_hpp_included

#include <boost/filesystem.hpp>

#include "math/math.hpp"
#include "math/transform.hpp"

#include "roarchive/roarchive.hpp"

#include "tinyxml2/tinyxml2.h"

namespace fs = boost::filesystem;
namespace xml = tinyxml2;

namespace lt {

const std::string mainXmlFileName("LODTreeExport.xml");

struct LodTreeNode
{
    double radius, minRange;
    math::Point3 origin;
    fs::path modelPath;
    std::vector<LodTreeNode> children;

    LodTreeNode(xml::XMLElement *elem, const fs::path &dir,
                const math::Point3 &rootOrigin);
};

struct LodTreeExport
{
    std::string srs;
    math::Point3 origin;
    std::vector<LodTreeNode> blocks;

    LodTreeExport(const roarchive::RoArchive &archive
            , const math::Point3 &offset);
};

}  // namespace lt

#endif // vts_tools_lodtreefile_hpp_included
