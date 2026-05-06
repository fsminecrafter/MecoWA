#include "scene_file.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace
{
    // Trim leading/trailing whitespace
    static std::string Trim(const std::string& s)
    {
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
    }

    // Split a line on ',' and trim each token
    static std::vector<std::string> SplitCSV(const std::string& line)
    {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ','))
            tokens.push_back(Trim(tok));
        return tokens;
    }

    // Safe float parse; returns fallback on failure
    static float ParseFloat(const std::string& s, float fallback = 0.f)
    {
        try   { return std::stof(s); }
        catch (...) { return fallback; }
    }

    // Advance past blank lines and comment lines; return the next data line
    // Returns false when EOF is reached.
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
}

// ─────────────────────────────────────────────────────────────────────────────
//  SceneFile::Load
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

    std::string line;
    int lineGroup = 0;   // which of the 4 lines we expect next

    SceneObject cur;

    auto commit = [&]()
    {
        objects.push_back(cur);
        cur = SceneObject{};
        lineGroup = 0;
    };

    while (NextDataLine(f, line))
    {
        auto tok = SplitCSV(line);

        switch (lineGroup)
        {
        // ── Line 1: name, parent ──────────────────────────────────────────
        case 0:
            if (tok.size() < 2)
            {
                std::cerr << "[SceneFile] Line 1 malformed: " << line << "\n";
                break;
            }
            cur.name   = tok[0];
            cur.parent = tok[1];
            lineGroup  = 1;
            break;

        // ── Line 2: modelPath, texturePath ────────────────────────────────
        case 1:
            if (tok.size() < 2)
            {
                std::cerr << "[SceneFile] Line 2 malformed: " << line << "\n";
                break;
            }
            cur.modelPath   = tok[0];
            cur.texturePath = tok[1];
            lineGroup       = 2;
            break;

        // ── Line 3: X Y Z RX RY RZ SX SY SZ ─────────────────────────────
        case 2:
            if (tok.size() < 9)
            {
                std::cerr << "[SceneFile] Line 3 malformed (need 9 values): " << line << "\n";
                break;
            }
            cur.x  = ParseFloat(tok[0]);
            cur.y  = ParseFloat(tok[1]);
            cur.z  = ParseFloat(tok[2]);
            cur.rx = ParseFloat(tok[3]);
            cur.ry = ParseFloat(tok[4]);
            cur.rz = ParseFloat(tok[5]);
            cur.sx = ParseFloat(tok[6], 1.f);
            cur.sy = ParseFloat(tok[7], 1.f);
            cur.sz = ParseFloat(tok[8], 1.f);
            lineGroup = 3;
            break;

        // ── Line 4: static, weight ────────────────────────────────────────
        case 3:
            if (tok.size() < 2)
            {
                std::cerr << "[SceneFile] Line 4 malformed: " << line << "\n";
                break;
            }
            cur.isStatic = (tok[0] == "1" || tok[0] == "true");
            cur.weight   = ParseFloat(tok[1]);
            commit();
            break;
        }
    }

    // Partial object at EOF
    if (lineGroup != 0)
        std::cerr << "[SceneFile] Warning: incomplete object '" << cur.name
                  << "' at end of file (stopped at line group " << lineGroup << ")\n";

    std::cout << "[SceneFile] Loaded " << objects.size()
              << " object(s) from " << path << "\n";
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  SceneFile::Save
// ─────────────────────────────────────────────────────────────────────────────
bool SceneFile::Save(const std::string& path) const
{
    std::ofstream f(path);
    if (!f.is_open())
    {
        std::cerr << "[SceneFile] Cannot write: " << path << "\n";
        return false;
    }

    f << "# MecoWA Scene File\n"
      << "# Format version 1\n"
      << "# ObjectName, ParentName\n"
      << "# ModelPath, TexturePath\n"
      << "# X, Y, Z, RX, RY, RZ, SX, SY, SZ\n"
      << "# Static(0/1), Weight(kg)\n\n";

    for (const auto& o : objects)
    {
        // Comment separator
        f << "# ── " << o.name << " ──\n";

        // Line 1
        f << o.name << ", " << o.parent << "\n";

        // Line 2
        f << o.modelPath << ", " << o.texturePath << "\n";

        // Line 3  (fixed 6-decimal precision for transforms)
        f << std::fixed;
        f.precision(6);
        f << o.x  << ", " << o.y  << ", " << o.z  << ", "
          << o.rx << ", " << o.ry << ", " << o.rz << ", "
          << o.sx << ", " << o.sy << ", " << o.sz << "\n";

        // Line 4
        f << (o.isStatic ? 1 : 0) << ", " << o.weight << "\n\n";
    }

    std::cout << "[SceneFile] Saved " << objects.size()
              << " object(s) to " << path << "\n";
    return true;
}
