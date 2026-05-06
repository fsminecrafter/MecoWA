/*  debugui_scenefile.cpp
 *
 *  Drop-in addition to the MecoWA ImGui debug overlay.
 *
 *  HOW TO INTEGRATE
 *  ────────────────
 *  1. Copy scene_file.h / scene_file.cpp into Core/Scripts/mainscript/
 *     and add both to MecoWA.vcxproj.
 *
 *  2. In debugui.h  add:
 *       void DebugUI_RenderSceneFilePanel();   // call from inside the tab bar
 *
 *  3. In debugui.cpp, inside DebugUI_Render() where the tab bar lives,
 *     add one more tab item:
 *       if (ImGui::BeginTabItem("Scene File"))
 *           { DebugUI_RenderSceneFilePanel(); ImGui::EndTabItem(); }
 *
 *  4. Add this file to the project (ClCompile in vcxproj).
 *
 *  The panel lets you:
 *    • Type / browse a .scn path
 *    • Load  – parse the file and populate the "pending" object list
 *    • Apply – push all loaded objects into the live scene via CreateObject()
 *              + RegisterPhysics_Box / RegisterPhysics_Convex
 *    • Save  – serialise the current sceneModels list back to a .scn file
 *    • Edit  – tweak every field of every pending object before applying
 *    • New   – add a blank object to the pending list
 *    • Clear – wipe the pending list
 */

#include "debugui.h"
#include "scene_file.h"
#include "engine.h"
#include "mecowa.h"

// Jolt / physics helpers already declared in engine.h
// void RegisterPhysics_Box(ModelInstance&, const OBJData&, float mass,
//                          float friction, float restitution,
//                          bool originAtBottom, glm::vec3 boxsize);
// void RegisterPhysics_Convex(ModelInstance&, float mass, ...);

#include <imgui/imgui.h>
#include <glm/glm/glm.hpp>

#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
//  Module state (file-local)
// ─────────────────────────────────────────────────────────────────────────────
namespace SceneUI
{
    // The path shown in the text box
    static char  s_pathBuf[512] = "myscene.scn";

    // Objects loaded from / to be saved to the file
    static SceneFile s_sceneFile;

    // Status message shown at the bottom of the panel
    static char  s_status[256] = "No file loaded.";
    static float s_statusTimer = 0.f;           // seconds to keep it lit
    static bool  s_statusOk    = true;          // green / red

    // Per-object editing state – mirrors s_sceneFile.objects but editable
    // Each entry maps to s_sceneFile.objects[i] via index.
    struct EditState
    {
        char name[64]        = {};
        char parent[64]      = {};
        char modelPath[256]  = {};
        char texturePath[256]= {};
        float transform[9]   = {0,0,0, 0,0,0, 1,1,1};  // xyzrxryrzsxsysz
        bool  isStatic       = false;
        float weight         = 0.f;
    };
    static std::vector<EditState> s_editStates;

    // ── Helpers ───────────────────────────────────────────────────────────────

    static void SetStatus(const char* msg, bool ok = true)
    {
        snprintf(s_status, sizeof(s_status), "%s", msg);
        s_statusOk    = ok;
        s_statusTimer = 5.f;
    }

    // Sync s_editStates from s_sceneFile.objects
    static void RebuildEditStates()
    {
        s_editStates.clear();
        s_editStates.reserve(s_sceneFile.objects.size());

        for (const auto& o : s_sceneFile.objects)
        {
            EditState e{};
            snprintf(e.name,        sizeof(e.name),        "%s", o.name.c_str());
            snprintf(e.parent,      sizeof(e.parent),      "%s", o.parent.c_str());
            snprintf(e.modelPath,   sizeof(e.modelPath),   "%s", o.modelPath.c_str());
            snprintf(e.texturePath, sizeof(e.texturePath), "%s", o.texturePath.c_str());
            e.transform[0] = o.x;  e.transform[1] = o.y;  e.transform[2] = o.z;
            e.transform[3] = o.rx; e.transform[4] = o.ry; e.transform[5] = o.rz;
            e.transform[6] = o.sx; e.transform[7] = o.sy; e.transform[8] = o.sz;
            e.isStatic = o.isStatic;
            e.weight   = o.weight;
            s_editStates.push_back(e);
        }
    }

    // Sync s_sceneFile.objects from s_editStates
    static void CommitEditStates()
    {
        s_sceneFile.objects.resize(s_editStates.size());
        for (size_t i = 0; i < s_editStates.size(); ++i)
        {
            const auto& e = s_editStates[i];
            auto& o = s_sceneFile.objects[i];
            o.name        = e.name;
            o.parent      = e.parent;
            o.modelPath   = e.modelPath;
            o.texturePath = e.texturePath;
            o.x  = e.transform[0]; o.y  = e.transform[1]; o.z  = e.transform[2];
            o.rx = e.transform[3]; o.ry = e.transform[4]; o.rz = e.transform[5];
            o.sx = e.transform[6]; o.sy = e.transform[7]; o.sz = e.transform[8];
            o.isStatic = e.isStatic;
            o.weight   = e.weight;
        }
    }

    // Spawn one SceneObject into the live scene
    static void SpawnObject(const SceneObject& o)
    {
        OBJData objData;

        // Build full path (backslash-friendly for Windows)
        std::string mp = o.modelPath;
        for (auto& c : mp) if (c == '/') c = '\\';

        ModelInstance& inst = CreateObject(
            mp, objData,
            o.name,
            glm::vec3(o.x,  o.y,  o.z),
            glm::vec3(o.rx, o.ry, o.rz),
            glm::vec3(o.sx, o.sy, o.sz)
        );

        // Physics registration
        float mass = o.weight;   // 0 = let the engine decide

        if (o.isStatic)
        {
            // Static objects: use box physics (floor, walls, etc.)
            RegisterPhysics_Box(inst, objData, 0.f,
                                0.8f, 0.1f, false,
                                glm::vec3(o.sx, o.sy, o.sz));
        }
        else if (mass <= 0.f)
        {
            // Dynamic with auto-mass → convex hull
            RegisterPhysics_Convex(inst, 1.f);
        }
        else
        {
            // Dynamic with explicit mass → box
            RegisterPhysics_Box(inst, objData, mass,
                                0.5f, 0.1f, false,
                                glm::vec3(o.sx, o.sy, o.sz));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public panel entry point
// ─────────────────────────────────────────────────────────────────────────────
void DebugUI_RenderSceneFilePanel()
{
    using namespace SceneUI;

    // Tick status timer
    if (s_statusTimer > 0.f) s_statusTimer -= ImGui::GetIO().DeltaTime;

    // ── File path row ─────────────────────────────────────────────────────────
    ImGui::SeparatorText("Scene File");

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 140.f);
    ImGui::InputText("##scnpath", s_pathBuf, sizeof(s_pathBuf));
    ImGui::SameLine();

    // Load button
    if (ImGui::Button("Load", {64, 0}))
    {
        if (s_sceneFile.Load(s_pathBuf))
        {
            RebuildEditStates();
            char msg[280];
            snprintf(msg, sizeof(msg), "Loaded %zu object(s) from '%s'.",
                     s_sceneFile.objects.size(), s_pathBuf);
            SetStatus(msg, true);
        }
        else
        {
            char msg[280];
            snprintf(msg, sizeof(msg), "Failed to load '%s'.", s_pathBuf);
            SetStatus(msg, false);
        }
    }

    ImGui::SameLine();

    // Save button – commits edits first
    if (ImGui::Button("Save", {64, 0}))
    {
        CommitEditStates();
        if (s_sceneFile.Save(s_pathBuf))
        {
            char msg[280];
            snprintf(msg, sizeof(msg), "Saved %zu object(s) to '%s'.",
                     s_sceneFile.objects.size(), s_pathBuf);
            SetStatus(msg, true);
        }
        else
        {
            char msg[280];
            snprintf(msg, sizeof(msg), "Failed to save '%s'.", s_pathBuf);
            SetStatus(msg, false);
        }
    }

    // ── Toolbar ───────────────────────────────────────────────────────────────
    ImGui::Spacing();

    // Apply: spawn all pending objects into the live scene
    ImGui::PushStyleColor(ImGuiCol_Button, {0.20f, 0.45f, 0.20f, 1.f});
    if (ImGui::Button("Apply to Scene"))
    {
        CommitEditStates();
        int spawned = 0;
        for (const auto& o : s_sceneFile.objects)
        {
            SpawnObject(o);
            ++spawned;
        }
        char msg[128];
        snprintf(msg, sizeof(msg), "Spawned %d object(s) into scene.", spawned);
        SetStatus(msg, true);
    }
    ImGui::PopStyleColor();

    ImGui::SameLine();

    // Save current live scene
    ImGui::PushStyleColor(ImGuiCol_Button, {0.20f, 0.25f, 0.50f, 1.f});
    if (ImGui::Button("Capture Live Scene"))
    {
        s_sceneFile.objects.clear();
        for (const auto& obj : sceneModels)
        {
            const ModelInstance& inst = obj.instance;
            SceneObject so;
            so.name        = obj.name;
            so.parent      = "none";
            so.modelPath   = "Core/Resources/3dmodels/" + obj.name + ".obj";
            so.texturePath = "none";
            so.x  = inst.position.x; so.y  = inst.position.y; so.z  = inst.position.z;
            so.rx = inst.rotation.x; so.ry = inst.rotation.y; so.rz = inst.rotation.z;
            so.sx = inst.scale.x;    so.sy = inst.scale.y;    so.sz = inst.scale.z;
            so.isStatic = false;
            so.weight   = 0.f;
            s_sceneFile.objects.push_back(so);
        }
        RebuildEditStates();
        char msg[128];
        snprintf(msg, sizeof(msg), "Captured %zu live object(s). Edit then Save.",
                 s_sceneFile.objects.size());
        SetStatus(msg, true);
    }
    ImGui::PopStyleColor();

    ImGui::SameLine();

    if (ImGui::Button("+ New Object"))
    {
        EditState e{};
        snprintf(e.name,        sizeof(e.name),        "NewObject");
        snprintf(e.parent,      sizeof(e.parent),      "none");
        snprintf(e.modelPath,   sizeof(e.modelPath),   "Core/Resources/3dmodels/cube.obj");
        snprintf(e.texturePath, sizeof(e.texturePath), "none");
        e.transform[6] = e.transform[7] = e.transform[8] = 1.f;  // scale = 1
        s_editStates.push_back(e);
        SetStatus("New object added. Fill in fields then Apply or Save.", true);
    }

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button, {0.50f, 0.10f, 0.10f, 1.f});
    if (ImGui::Button("Clear"))
    {
        s_sceneFile.Clear();
        s_editStates.clear();
        SetStatus("Pending list cleared.", true);
    }
    ImGui::PopStyleColor();

    // ── Object editor table ───────────────────────────────────────────────────
    ImGui::Spacing();
    ImGui::Text("Pending objects: %zu", s_editStates.size());
    ImGui::Separator();

    // Scrollable child so the status bar stays pinned at the bottom
    float reserveBottom = 40.f;
    ImGui::BeginChild("##scnobjs",
        ImVec2(0.f, ImGui::GetContentRegionAvail().y - reserveBottom),
        false, ImGuiWindowFlags_HorizontalScrollbar);

    int toDelete = -1;

    for (int i = 0; i < (int)s_editStates.size(); ++i)
    {
        auto& e = s_editStates[i];

        // Collapsible per-object header
        char header[128];
        snprintf(header, sizeof(header), "[%d]  %s  (%s)  %s##hdr%d",
                 i,
                 e.name,
                 e.isStatic ? "static" : "dynamic",
                 (e.weight > 0.f) ? "" : "auto-mass",
                 i);

        bool open = ImGui::TreeNodeEx(header, ImGuiTreeNodeFlags_SpanFullWidth);

        // Inline delete button on the right
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 50.f +
                        ImGui::GetCursorPosX());
        char delId[32]; snprintf(delId, sizeof(delId), "Del##d%d", i);
        ImGui::PushStyleColor(ImGuiCol_Button, {0.5f, 0.1f, 0.1f, 1.f});
        if (ImGui::SmallButton(delId)) toDelete = i;
        ImGui::PopStyleColor();

        if (!open) continue;

        ImGui::PushID(i);

        // ── Identity ─────────────────────────────────────────────────────────
        ImGui::SeparatorText("Identity");
        ImGui::SetNextItemWidth(180);
        ImGui::InputText("Name##n",   e.name,   sizeof(e.name));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(180);
        ImGui::InputText("Parent##p", e.parent, sizeof(e.parent));

        // ── Assets ───────────────────────────────────────────────────────────
        ImGui::SeparatorText("Assets  (relative to project root)");
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.7f);
        ImGui::InputText("Model##m",   e.modelPath,   sizeof(e.modelPath));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.7f);
        ImGui::InputText("Texture##t", e.texturePath, sizeof(e.texturePath));

        // ── Transform ────────────────────────────────────────────────────────
        ImGui::SeparatorText("Transform");

        ImGui::Text("Position");
        ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat3("##pos", e.transform + 0, 0.01f, -1e4f, 1e4f, "%.3f");

        ImGui::Text("Rotation");
        ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat3("##rot", e.transform + 3, 0.1f, -360.f, 360.f, "%.2f°");

        ImGui::Text("Scale");
        ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        ImGui::DragFloat3("##scl", e.transform + 6, 0.005f, 0.001f, 100.f, "%.3f");

        // ── Physics ──────────────────────────────────────────────────────────
        ImGui::SeparatorText("Physics");
        ImGui::Checkbox("Static", &e.isStatic);
        ImGui::SameLine(110);
        ImGui::SetNextItemWidth(120);
        ImGui::DragFloat("Weight (kg)##w", &e.weight, 0.1f, 0.f, 1e6f, "%.1f");
        if (e.weight <= 0.f)
            ImGui::SameLine(), ImGui::TextDisabled("(auto)");

        ImGui::PopID();
        ImGui::TreePop();
    }

    // Deferred delete (avoids iterator invalidation inside the loop)
    if (toDelete >= 0 && toDelete < (int)s_editStates.size())
    {
        s_editStates.erase(s_editStates.begin() + toDelete);
        SetStatus("Object removed from pending list.", true);
    }

    ImGui::EndChild();

    // ── Status bar ────────────────────────────────────────────────────────────
    ImGui::Separator();
    if (s_statusTimer > 0.f)
    {
        ImVec4 col = s_statusOk
            ? ImVec4{0.30f, 1.00f, 0.40f, 1.f}
            : ImVec4{1.00f, 0.35f, 0.35f, 1.f};
        ImGui::PushStyleColor(ImGuiCol_Text, col);
        ImGui::TextUnformatted(s_status);
        ImGui::PopStyleColor();
    }
    else
    {
        ImGui::TextDisabled("%s", s_status);
    }
}
