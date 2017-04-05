#ifndef vts_tools_lodtreefile_hpp_included
#define vts_tools_lodtreefile_hpp_included

#include <boost/filesystem.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <opencv2/highgui/highgui.hpp>

#include <tinyxml2.h>

#include "math/math.hpp"
#include "math/transform.hpp"

#include "roarchive/roarchive.hpp"

namespace fs = boost::filesystem;
namespace xml = tinyxml2;

namespace lt {

const std::string mainXmlFileName("LODTreeExport.xml");

const aiScene* readScene(Assimp::Importer &imp
            , const roarchive::RoArchive &archive
            , const fs::path &path
            , unsigned int flags);

cv::Mat readTexture(const roarchive::RoArchive &archive, const fs::path &path
        , bool useEmpty = false);

math::Point3 point3(const aiVector3D &vec);

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
