#ifndef lodtree_lodtreefile_hpp_included
#define lodtree_lodtreefile_hpp_included

#include <boost/filesystem.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <tinyxml2.h>

#include "math/math.hpp"
#include "math/transform.hpp"

#include "roarchive/roarchive.hpp"

#include "geo/srsdef.hpp"

namespace lodtree {

const std::string mainXmlFileName("LODTreeExport.xml");
const std::string alternativeXmlFileName("metadata.xml");

const aiScene* readScene(Assimp::Importer &imp
            , const roarchive::RoArchive &archive
            , const boost::filesystem::path &path
            , unsigned int flags);

cv::Mat readTexture(const roarchive::RoArchive &archive
                    , const boost::filesystem::path &path
                    , bool useEmpty = false);

math::Point3 point3(const aiVector3D &vec);

struct LodTreeNode
{
    double radius, minRange;
    math::Point3 origin;
    boost::filesystem::path modelPath;
    std::vector<LodTreeNode> children;

    LodTreeNode(tinyxml2::XMLElement *elem, const boost::filesystem::path &dir,
                const math::Point3 &rootOrigin);

    LodTreeNode(const boost::filesystem::path &modelPath
                , const math::Point3 &origin)
        : radius(), minRange(), origin(origin), modelPath(modelPath)
    {}
};

struct LodTreeExport
{
    geo::SrsDefinition srs;
    math::Point3 origin;
    std::vector<LodTreeNode> blocks;

    LodTreeExport(const roarchive::RoArchive &archive
            , const math::Point3 &offset);
};

}  // namespace lodtree

#endif // lodtree_lodtreefile_hpp_included
