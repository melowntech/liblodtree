#include "lodtreefile.hpp"

namespace lt {

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

LodTreeNode::LodTreeNode(tinyxml2::XMLElement *node, const fs::path &dir,
                         const math::Point3 &rootOrigin)
{
    int ok = xml::XML_SUCCESS;
    if (getElement(node, "Radius")->QueryDoubleText(&radius) != ok ||
        getElement(node, "MinRange")->QueryDoubleText(&minRange) != ok)
    {
        LOGTHROW(err3, std::runtime_error) << "Error reading node data";
    }

    auto *ctr = getElement(node, "Center");
    math::Point3 center(getDoubleAttr(ctr, "x"),
                        getDoubleAttr(ctr, "y"),
                        getDoubleAttr(ctr, "z"));
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
            children.emplace_back(elem, dir, rootOrigin);
        }
    }
}

LodTreeExport::LodTreeExport(const roarchive::RoArchive &archive
        , const math::Point3 &offset)
{
    xml::XMLDocument doc;
    auto *root = loadLodTreeXml(archive, mainXmlFileName, doc);

    srs = getElement(root, "SRS")->GetText();

    auto *local = getElement(root, "Local");
    origin(0) = getDoubleAttr(local, "x");
    origin(1) = getDoubleAttr(local, "y");
    origin(2) = getDoubleAttr(local, "z");
    origin += offset;

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

            blocks.emplace_back(rootNode, path.parent_path(), origin);
        }
    }
}

}