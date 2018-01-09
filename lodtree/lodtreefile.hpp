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

struct Node
{
    typedef std::vector<Node> list;

    double radius, minRange;
    math::Point3 origin;
    boost::filesystem::path modelPath;
    list children;
    int level;

    Node(tinyxml2::XMLElement *elem, const boost::filesystem::path &dir
         , const math::Point3 &rootOrigin, int level);

    Node(const boost::filesystem::path &modelPath
         , const math::Point3 &origin
         , int level)
        : radius(), minRange(), origin(origin), modelPath(modelPath)
        , level(level)
    {}
};

class LodTreeExport
{
public:
    geo::SrsDefinition srs;
    math::Point3 origin;
    Node::list blocks;

    LodTreeExport(roarchive::RoArchive &archive
                  , const math::Point3 &offset);

    LodTreeExport(const boost::filesystem::path &root
                  , const math::Point3 &offset
                  , const std::string &mime = "");

    /** Returns lodtree as flat node list.
     */
    Node::list nodes() const;

    const roarchive::RoArchive &archive() const { return archive_; }

private:
    roarchive::RoArchive archive_;
};

/** For compatibility.
 */
typedef Node LodTreeNode;

}  // namespace lodtree

#endif // lodtree_lodtreefile_hpp_included
