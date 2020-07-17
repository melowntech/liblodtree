#include <iterator>
#include <stack>

#include <boost/range/adaptor/map.hpp>
#include <boost/range/irange.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <ogr_spatialref.h>

#include "utility/streams.hpp"

#include "geo/enu.hpp"

#include "imgproc/readimage.hpp"
#include "imgproc/cvcompat.hpp"

#include "lodtreefile.hpp"

namespace fs = boost::filesystem;
namespace xml = tinyxml2;
namespace ba = boost::algorithm;

namespace lodtree {

math::Point3 point3(const aiVector3D &vec) {
    return {vec.x, vec.y, vec.z};
}

const aiScene* readScene(Assimp::Importer &imp
        , const roarchive::RoArchive &archive
        , const fs::path &path
        , unsigned int flags)
{
    // if (archive.directio()) {
    //     return imp.ReadFile(archive.path(path).string(), flags);
    // }

    const auto buf(archive.istream(path)->read());

    const auto *scene
            (imp.ReadFileFromMemory(buf.data(), buf.size(), flags));
    // , path.extension().c_str()));
    if (!scene) {
        LOGTHROW(err3, std::runtime_error)
                << "Error loading scene " << path
                << "( " << imp.GetErrorString() << " ).";
    }

    return scene;
}

cv::Mat readTexture(const roarchive::RoArchive &archive, const fs::path &path
        , bool useEmpty)
{
    cv::Mat texture;
    // if (archive.directio()) {
    //     texture = cv::imread(archive.path(path).string(), CV_LOAD_IMAGE_COLOR);
    // } else {
    const auto buf(archive.istream(path)->read());
    texture = cv::imdecode(buf, IMGPROC_IMREAD(COLOR));
    // }

    if (texture.data) { return texture; }

    if (!useEmpty) {
        LOGTHROW(err3, std::runtime_error)
                << "Error loading texture from " << path << ".";
    }

    LOG(warn3)
        << "Error loading image " << path << "; using empty texture.";
    texture.create(64, 64, CV_8UC3);
    texture = cv::Scalar(255, 255, 255);

    return texture;
}

xml::XMLError readXml(const roarchive::RoArchive &archive
        , const fs::path &path, xml::XMLDocument &doc)
{
    // if (archive.directio()) {
    //     return doc.LoadFile(archive.path(path).c_str());
    // }

    const auto buf(archive.istream(path)->read());
    return doc.Parse(buf.data(), buf.size());
}

xml::XMLElement* getElement(xml::XMLNode *node, const char* elemName)
{
    xml::XMLElement* elem = node->FirstChildElement(elemName);
    if (!elem) {
        LOGTHROW(err3, std::runtime_error)
                << "XML element \"" << elemName << "\" not found.";
    }
    return elem;
}

void errorAttrNotFound(xml::XMLElement *elem, const char* attrName)
{
    LOGTHROW(err3, std::runtime_error)
            << "XML attribute \"" << attrName
            << "\" not found in element \"" << elem->Name() << "\".";
}

const char* getTextAttr(xml::XMLElement *elem, const char* attrName)
{
    const char* text = elem->Attribute(attrName);
    if (!text) {
        errorAttrNotFound(elem, attrName);
    }
    return text;
}

double getDoubleAttr(xml::XMLElement *elem, const char* attrName)
{
    double a;
    if (elem->QueryDoubleAttribute(attrName, &a) == xml::XML_NO_ATTRIBUTE) {
        errorAttrNotFound(elem, attrName);
    }
    return a;
}

math::Point3d point(xml::XMLElement *element)
{
    return  { getDoubleAttr(element, "x")
            , getDoubleAttr(element, "y")
            , getDoubleAttr(element, "z") };
}

xml::XMLElement* loadLodTreeXml(const roarchive::RoArchive &archive
        , const fs::path &fname
        , xml::XMLDocument &doc)
{
    auto err = readXml(archive, fname, doc);
    if (err != xml::XML_SUCCESS) {
        LOGTHROW(err3, std::runtime_error)
                << "Error loading " << fname << ": " << doc.ErrorName();
    }

    auto *root = getElement(&doc, "LODTreeExport");

    double version = getDoubleAttr(root, "version");
    if (version > 1.1 + 1e-12) {
        LOGTHROW(err3, std::runtime_error)
                << fname << ": unsupported format version (" << version << ").";
    }

    return root;
}

Node::Node(tinyxml2::XMLElement *node, const fs::path &dir
           , const math::Point3 &rootOrigin, int level)
    : level(level)
{
    int ok = xml::XML_SUCCESS;
    if (getElement(node, "Radius")->QueryDoubleText(&radius) != ok ||
        getElement(node, "MinRange")->QueryDoubleText(&minRange) != ok)
    {
        LOGTHROW(err3, std::runtime_error) << "Error reading node data";
    }

    const auto center(point(getElement(node, "Center")));
    origin = rootOrigin + center;

    auto* mpath = node->FirstChildElement("ModelPath");
    if (mpath) {
        modelPath = dir / mpath->GetText();
    }

    // load all children
    std::string strNode("Node");
    for (auto *elem = node->FirstChildElement();
         elem;
         elem = elem->NextSiblingElement())
    {
        if (strNode == elem->Name())
        {
            children.emplace_back(elem, dir, rootOrigin, level + 1);
        }
    }
}

namespace {

void buildParams(LodTreeExport &lte, const std::string &srsString
                 , const math::Point3 &srsOrigin)
{
    lte.origin = srsOrigin;

    if (!srsString.empty()) {
        // parse SRS
        lte.srs = geo::SrsDefinition::fromString(srsString);

        if (lte.srs.reference().IsGeographic()) {
            // non-metric system -> ENU
            lte.srs = geo::SrsDefinition::fromEnu
                (geo::Enu(lte.origin, lte.srs));
            // reset origin
            lte.origin = {};
        }
    }

    LOG(info3) << std::fixed
               << "LODTree: SRS=\"" << lte.srs
               << "\", origin: " << lte.origin;
}

bool openArchive(const roarchive::RoArchive &archive
                 , const math::Point3 &offset
                 , LodTreeExport &lte)
{
    xml::XMLDocument doc;
    xml::XMLElement *root(nullptr);
    try {
        root = loadLodTreeXml(archive, mainXmlFileName, doc);
    } catch (const roarchive::Error &e) {
        return false;
    }

    // get origin point
    buildParams(lte, getElement(root, "SRS")->GetText()
                , point(getElement(root, "Local")));
    // shift by offset
    lte.origin += offset;

    // load all blocks ("Tiles")
    std::string strTile("Tile");
    for (auto *elem = root->FirstChildElement();
         elem;
         elem = elem->NextSiblingElement())
    {
        if (strTile == elem->Name())
        {
            fs::path path(getTextAttr(elem, "path"));
            LOG(info3) << "Parsing block " << path << ".";

            xml::XMLDocument tileDoc;
            auto *tileRoot = loadLodTreeXml(archive, path, tileDoc);
            auto *rootNode = getElement(tileRoot, "Tile");

            lte.blocks.emplace_back
                (rootNode, path.parent_path(), lte.origin, 0);
        }
    }

    return true;
}

xml::XMLElement* loadMetadataXml(const roarchive::RoArchive &archive
                                 , const fs::path &fname
                                 , xml::XMLDocument &doc)
{
    auto err = readXml(archive, fname, doc);
    if (err != xml::XML_SUCCESS) {
        LOGTHROW(err3, std::runtime_error)
            << "Error loading " << fname << ": " << doc.ErrorName();
    }

    auto *root = getElement(&doc, "ModelMetadata");

    double version = getDoubleAttr(root, "version");
    if (version > 1.0 + 1e-12) {
        LOGTHROW(err3, std::runtime_error)
            << fname << ": unsupported format version (" << version << ").";
    }

    return root;
}

math::Point3d point(const std::string &def)
{
    math::Point3 p;
    std::istringstream is(def);
    is >> p(0)
       >> utility::expect(',') >> p(1)
       >> utility::expect(',') >> p(2)
        ;
    return p;
}

struct DirEntry {
    typedef std::map<std::string, DirEntry> Children;

    DirEntry() {}
    DirEntry(const std::string &name, const fs::path &path)
        : path(path), name(name)
    {}

    fs::path path;
    std::string name;
    Children children;

    bool directory() const { return !children.empty(); }

    void add(const fs::path &path, const fs::path::iterator &begin
             , const fs::path::iterator &end)
    {
        if (begin == end) { return; }

        const auto name(begin->string());
        const auto self(path / name);

        auto fchildren(children.find(name));
        if (fchildren == children.end()) {
            fchildren = children.insert
                (Children::value_type(name, DirEntry(name, self))).first;
        }

        fchildren->second.add(self, std::next(begin), end);
    }

    void add(const fs::path &path) {
        add({}, path.begin(), path.end());
    }

    void dump(const std::string &prefix = "") const {
        LOG(info4) << prefix << name;
        for (const auto &item : children) {
            item.second.dump(prefix + "    ");
        }
    }

    const DirEntry* operator[](const std::string &name) const {
        auto fchildren(children.find(name));
        if (fchildren == children.end()) { return nullptr; }
        return &fchildren->second;
    }
};

auto begin(const DirEntry &de)
    -> decltype(std::begin(boost::adaptors::values(de.children)))
{
    return std::begin(boost::adaptors::values(de.children));
}

auto end(const DirEntry &de)
    -> decltype(std::end(boost::adaptors::values(de.children)))
{
    return std::end(boost::adaptors::values(de.children));
}

typedef boost::iterator_range<std::string::const_iterator> Token;
typedef std::vector<Token> Tokens;

std::string string(const Token &token)
{
    return { std::begin(token), std::end(token) };
}

const std::vector<std::string> meshExtensions{ ".obj", ".dae" };

bool isMesh(const std::string &fname)
{
    for (const auto &ext : meshExtensions) {
        if (ba::iends_with(fname, ext)) { return true; }
    }
    return false;
}

class TreeBuilder {
public:
    TreeBuilder() {}

    void add(const std::string &id, const fs::path &path) {
        nodes_.emplace_back(id, path);
    }

    void getTree(std::vector<Node> &roots
                 , const math::Point3 &origin);

private:
    struct TBNode {
        std::string id;
        fs::path path;

        TBNode(const std::string &id, const fs::path &path)
            : id(id), path(path)
        {}

        bool operator<(const TBNode &o) const {
            return id < o.id;
        }

        typedef std::vector<TBNode> list;
    };

    TBNode::list nodes_;
};

void TreeBuilder::getTree(std::vector<Node> &roots
                          , const math::Point3 &origin)
{
    if (nodes_.empty()) { return; }

    std::sort(nodes_.begin(), nodes_.end());

    auto inodes(nodes_.begin());
    auto enodes(nodes_.end());

    roots.emplace_back(inodes->path, origin, 0);

    struct NodeInfo {
        std::string id;
        Node *node;

        NodeInfo(const std::string &id, Node *node)
            : id(id), node(node)
        {}

        typedef std::stack<NodeInfo> stack;
    };

    // create node info stack and push new root
    NodeInfo::stack ns;
    ns.emplace(inodes->id, &roots.back());

    for (++inodes; inodes != enodes; ++inodes) {
        const auto &newId(inodes->id);

        // fix-up stack to be just one below
        while (ns.top().id.size() >= newId.size()) { ns.pop(); }

        // create new node
        auto &top(ns.top());

        if (newId.compare(0, top.id.size(), top.id)) {
            LOG(warn3) << "Node <" << newId << "> doesn't fit under "
                "upper node <" << top.id << ">; skipping.";
            continue;
        }

        top.node->children.emplace_back(inodes->path, origin, newId.size());
        ns.emplace(newId, &top.node->children.back());
    }
}

void openPseudoArchive(const roarchive::RoArchive &archive
                       , const math::Point3 &offset
                       , LodTreeExport &lte)
{
    {
        xml::XMLDocument doc;
        auto *root(loadMetadataXml(archive, alternativeXmlFileName, doc));

        buildParams(lte, getElement(root, "SRS")->GetText()
                    , point(getElement(root, "SRSOrigin")->GetText()));
    }

    // shift by offset
    lte.origin += offset;

    // process files in the archive
    DirEntry root;
    for (const auto &file : archive.list()) { root.add(file); }

    const auto *Data(root["Data"]);
    if (!Data) {
        LOGTHROW(err3, std::runtime_error)
            << "Cannot find Data directory in the archive.";
    }


    for (const auto &dir : *Data) {
        if (!ba::starts_with(dir.name, "Tile_")) { continue; }

        TreeBuilder builder;

        // process tile dir
        LOG(info1) << "Scanning tile dir " << dir.path << ".";
        for (const auto &file : dir) {
            if (!ba::starts_with(file.name, dir.name)) { continue; }
            if (!isMesh(file.name)) { continue; }

            Tokens tokens;
            const auto range
                (boost::make_iterator_range
                 (file.name.begin() + dir.name.size(), file.name.end()));
            ba::split(tokens, range, ba::is_any_of("_."));

            if (tokens.size() == 3) {
                // root
                builder.add({}, file.path);
            } else if (tokens.size() == 4) {
                // some child
                builder.add(string(tokens[2]), file.path);
            }

        }

        builder.getTree(lte.blocks, lte.origin);
    }
}

} // namespace

LodTreeExport::LodTreeExport(roarchive::RoArchive &archive
                             , const math::Point3 &offset)
    : archive_(archive.applyHint({ lodtree::mainXmlFileName
                    , lodtree::alternativeXmlFileName }))
{
    if (!openArchive(archive_, offset, *this)) {
        openPseudoArchive(archive_, offset, *this);
    }
}

LodTreeExport::LodTreeExport(const boost::filesystem::path &root
                             , const math::Point3 &offset
                             , const std::string &mime)
    : archive_
      (root, roarchive::OpenOptions()
       .setHint({ lodtree::mainXmlFileName
                   , lodtree::alternativeXmlFileName })
       .setMime(mime))
{
    if (!openArchive(archive_, offset, *this)) {
        openPseudoArchive(archive_, offset, *this);
    }
}

namespace {

inline void addNodeTo(Node::list &nodes, const Node &node)
{
    nodes.push_back(node);
    for (const auto &child : node.children) {
        addNodeTo(nodes, child);
    }
}

} // namespace

Node::list LodTreeExport::nodes() const
{
    Node::list nodes;
    for (const auto &root : blocks) { addNodeTo(nodes, root); }
    return nodes;
}

} // namespace lodtree
