#include "scene_file.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace
{
    static std::string Trim(const std::string& s)
    {
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    }

    static std::vector<std::string> SplitCSV(const std::string& line)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ','))
            tokens.push_back(Trim(tok));
        return tokens;
    }

    static float ParseFloat(const std::string& s, float fallback = 0.f)
    {
        try   { return std::stof(s); }
        catch (...) { return fallback; }
    }

    static bool ParseBool(const std::string& s)
    {
        return (s == "1" || s == "true");
    }

    // Advance past blank lines and comment lines (# prefix)
    static bool NextDataLine(std::ifstream& f, std::string& out)
    {
        while (std::getline(f, out))
        {
            std::string t = Trim(out);
            if (!t.empty() && t[0] != '#')
            {
                out = t;
                return true;
            }
        }
        return false;
    }

    static ColliderShape ParseColliderShape(const std::string& s)
    {
        if (s == "Sphere")       return ColliderShape::Sphere;
        if (s == "Capsule")      return ColliderShape::Capsule;
        if (s == "ConvexHull")   return ColliderShape::ConvexHull;
        if (s == "TriangleMesh") return ColliderShape::TriangleMesh;
        return ColliderShape::Box;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  File format (v2)
//
//  # comment
//
//  OBJECT
//  name, parent
//  modelPath, texturePath
//  X, Y, Z, RX, RY, RZ, SX, SY, SZ
//  static(0/1), weight
//  collider_count
//  [for each collider:]
//  COLLIDER
//  name, shape, isTrigger(0/1), enabled(0/1)
//  ox, oy, oz, rx, ry, rz
//  sx, sy, sz
//  friction, restitution
//  END_COLLIDER
//  END_OBJECT
//
// ─────────────────────────────────────────────────────────────────────────────

bool SceneFile::Load(const std::string& path)
{
    objects.clear();

    std::ifstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[SceneFile] Cannot open: " << path << "\n";
        return false;
    }

    currentPath = path;
    m_modified  = false;

    std::string line;
    SceneObject cur;
    bool inObject    = false;
    bool inCollider  = false;
    SceneCollider cc;

    // Simple state-machine parser
    // States: 0=top, 1=got_name_parent, 2=got_assets, 3=got_transform,
    //         4=got_physics, 5=reading_colliders
    int objState = 0;
    int colState = 0;

    auto CommitCollider = [&]() {
        cur.colliders.push_back(cc);
        cc = SceneCollider{};
        colState = 0;
        inCollider = false;
    };

    auto CommitObject = [&]() {
        objects.push_back(cur);
        cur = SceneObject{};
        inObject = false;
        objState = 0;
    };

    while (std::getline(f, line))
    {
        std::string t = Trim(line);
        if (t.empty() || t[0] == '#') continue;

        // Top-level or object-level keywords
        if (t == "OBJECT")
        {
            inObject = true;
            objState = 0;
            cur = SceneObject{};
            continue;
        }
        if (t == "END_OBJECT")
        {
            if (inObject) CommitObject();
            continue;
        }
        if (t == "COLLIDER")
        {
            inCollider = true;
            colState   = 0;
            cc = SceneCollider{};
            continue;
        }
        if (t == "END_COLLIDER")
        {
            if (inCollider) CommitCollider();
            continue;
        }

        if (!inObject) continue;

        if (inCollider)
        {
            auto tok = SplitCSV(t);
            switch (colState)
            {
            case 0: // name, shape, isTrigger, enabled
                if (tok.size() >= 4)
                {
                    cc.name      = tok[0];
                    cc.shape     = ParseColliderShape(tok[1]);
                    cc.isTrigger = ParseBool(tok[2]);
                    cc.enabled   = ParseBool(tok[3]);
                }
                colState = 1;
                break;
            case 1: // ox, oy, oz, rx, ry, rz
                if (tok.size() >= 6)
                {
                    cc.ox = ParseFloat(tok[0]); cc.oy = ParseFloat(tok[1]); cc.oz = ParseFloat(tok[2]);
                    cc.rx = ParseFloat(tok[3]); cc.ry = ParseFloat(tok[4]); cc.rz = ParseFloat(tok[5]);
                }
                colState = 2;
                break;
            case 2: // sx, sy, sz
                if (tok.size() >= 3)
                {
                    cc.sx = ParseFloat(tok[0], 0.5f);
                    cc.sy = ParseFloat(tok[1], 0.5f);
                    cc.sz = ParseFloat(tok[2], 0.5f);
                }
                colState = 3;
                break;
            case 3: // friction, restitution
                if (tok.size() >= 2)
                {
                    cc.friction    = ParseFloat(tok[0], -1.f);
                    cc.restitution = ParseFloat(tok[1], -1.f);
                }
                colState = 4;
                break;
            }
            continue;
        }

        // Object field lines
        auto tok = SplitCSV(t);
        switch (objState)
        {
        case 0: // name, parent
            if (tok.size() >= 2)
            {
                cur.name   = tok[0];
                cur.parent = tok[1];
            }
            objState = 1;
            break;
        case 1: // modelPath, texturePath
            if (tok.size() >= 2)
            {
                cur.modelPath   = tok[0];
                cur.texturePath = tok[1];
            }
            objState = 2;
            break;
        case 2: // X Y Z RX RY RZ SX SY SZ
            if (tok.size() >= 9)
            {
                cur.x  = ParseFloat(tok[0]); cur.y  = ParseFloat(tok[1]); cur.z  = ParseFloat(tok[2]);
                cur.rx = ParseFloat(tok[3]); cur.ry = ParseFloat(tok[4]); cur.rz = ParseFloat(tok[5]);
                cur.sx = ParseFloat(tok[6], 1.f);
                cur.sy = ParseFloat(tok[7], 1.f);
                cur.sz = ParseFloat(tok[8], 1.f);
            }
            objState = 3;
            break;
        case 3: // static, weight
            if (tok.size() >= 2)
            {
                cur.isStatic = ParseBool(tok[0]);
                cur.weight   = ParseFloat(tok[1]);
            }
            objState = 4;
            break;
        // case 4+: colliders are handled by COLLIDER / END_COLLIDER blocks
        }
    }

    std::cout << "[SceneFile] Loaded " << objects.size()
              << " object(s) from " << path << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool SceneFile::Save(const std::string& path) const
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[SceneFile] Cannot write: " << path << "\n";
        return false;
    }

    f << "# MecoWA Scene File  (format v2)\n"
      << "# OBJECT / END_OBJECT blocks, COLLIDER / END_COLLIDER children\n\n";

    f << std::fixed;
    f.precision(6);

    for (const auto& o : objects)
    {
        f << "# ── " << o.name << " ──────────────────────────────────\n";
        f << "OBJECT\n";
        f << o.name        << ", " << o.parent      << "\n";
        f << o.modelPath   << ", " << o.texturePath  << "\n";
        f << o.x  << ", " << o.y  << ", " << o.z  << ", "
          << o.rx << ", " << o.ry << ", " << o.rz << ", "
          << o.sx << ", " << o.sy << ", " << o.sz << "\n";
        f << (o.isStatic ? 1 : 0) << ", " << o.weight << "\n";

        for (const auto& c : o.colliders)
        {
            f << "COLLIDER\n";
            f << c.name << ", " << ColliderShapeName(c.shape)
              << ", " << (c.isTrigger ? 1 : 0)
              << ", " << (c.enabled   ? 1 : 0) << "\n";
            f << c.ox << ", " << c.oy << ", " << c.oz << ", "
              << c.rx << ", " << c.ry << ", " << c.rz << "\n";
            f << c.sx << ", " << c.sy << ", " << c.sz << "\n";
            f << c.friction << ", " << c.restitution << "\n";
            f << "END_COLLIDER\n";
        }

        f << "END_OBJECT\n\n";
    }

    std::cout << "[SceneFile] Saved " << objects.size()
              << " object(s) to " << path << "\n";
    return true;
}
