#include "debugui.h"
#include "scene_file.h"
#include "engine.h"
#include "mecowa.h"

#include <imgui/imgui.h>

#include <glm/glm/glm.hpp>

#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
//  Forward declarations for engine helpers (defined in engine.cpp)
// ─────────────────────────────────────────────────────────────────────────────
// void RegisterPhysics_Box   (ModelInstance&, const OBJData&, float mass,
//                              float friction, float restitution,
//                              bool originAtBottom, glm::vec3 boxsize);
// void RegisterPhysics_Convex(ModelInstance&, float mass,
//                              float friction, float restitution, bool originAtBottom);

// ═════════════════════════════════════════════════════════════════════════════
//  Module state
// ═════════════════════════════════════════════════════════════════════════════
namespace SceneEd
{

    // ── File state ────────────────────────────────────────────────────────────────
    static SceneFile    g_scene;
    static char         g_pathBuf[512] = "scene.scn";
    static bool         g_showSaveAsPopup = false;
    static bool         g_showNewConfirm = false;
    static bool         g_showLoadConfirm = false;
    static char         g_pendingLoadPath[512] = {};

    // ── Selection ─────────────────────────────────────────────────────────────────
    static int          g_selObj = -1;   // selected object index
    static int          g_selCol = -1;   // selected collider index (-1 = none)

    // ── Status bar ────────────────────────────────────────────────────────────────
    static char         g_status[256] = "Ready.";
    static float        g_statusTimer = 0.f;
    static bool         g_statusOk = true;

    // ── Drag-reorder state ────────────────────────────────────────────────────────
    static int          g_dragSrc = -1;

    static void SetStatus(const char* msg, bool ok = true)
    {
        snprintf(g_status, sizeof(g_status), "%s", msg);
        g_statusOk = ok;
        g_statusTimer = 5.f;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Helpers
    // ─────────────────────────────────────────────────────────────────────────────

    // Clamp selection after deletion / reorder
    static void ClampSel()
    {
        int n = (int)g_scene.objects.size();
        if (g_selObj >= n) g_selObj = n - 1;
        if (g_selObj < 0 && n > 0) g_selObj = 0;
        if (n == 0) { g_selObj = -1; g_selCol = -1; }
        else
        {
            int nc = (int)g_scene.objects[g_selObj].colliders.size();
            if (g_selCol >= nc) g_selCol = nc - 1;
        }
    }

    // Build a list of valid parent names (all objects except current)
    static std::vector<std::string> ValidParents(int selfIdx)
    {
        std::vector<std::string> v;
        v.push_back("none");
        for (int i = 0; i < (int)g_scene.objects.size(); ++i)
            if (i != selfIdx)
                v.push_back(g_scene.objects[i].name);
        return v;
    }

    // Spawn a SceneObject into the live engine scene
    static void SpawnObject(const SceneObject& o)
    {
        OBJData objData;
        std::string mp = o.modelPath;
        for (auto& c : mp) if (c == '/') c = '\\';

        ModelInstance& inst = CreateObject(
            mp, objData, o.name,
            glm::vec3(o.x, o.y, o.z),
            glm::vec3(o.rx, o.ry, o.rz),
            glm::vec3(o.sx, o.sy, o.sz));

        // Determine physics registration from colliders
        bool hasBox = false;
        bool hasConvex = false;
        glm::vec3 boxHalf = { o.sx * 0.5f, o.sy * 0.5f, o.sz * 0.5f };
        float friction = 0.5f;
        float restitution = 0.1f;

        for (const auto& col : o.colliders)
        {
            if (!col.enabled || col.isTrigger) continue;
            if (col.friction >= 0.f) friction = col.friction;
            if (col.restitution >= 0.f) restitution = col.restitution;

            switch (col.shape)
            {
            case ColliderShape::Box:
                hasBox = true;
                boxHalf = { col.sx, col.sy, col.sz };
                break;
            case ColliderShape::ConvexHull:
            case ColliderShape::TriangleMesh:
            case ColliderShape::Sphere:
            case ColliderShape::Capsule:
                hasConvex = true;
                break;
            }
        }

        float mass = o.weight > 0.f ? o.weight : 1.f;

        if (o.isStatic)
        {
            RegisterPhysics_Box(inst, objData, 0.f, friction, restitution,
                false, glm::vec3(o.sx, o.sy, o.sz));
        }
        else if (hasConvex || (!hasBox && !hasConvex))
        {
            RegisterPhysics_Convex(inst, mass, friction, restitution, false);
        }
        else
        {
            RegisterPhysics_Box(inst, objData, mass, friction, restitution,
                false, boxHalf * 2.f);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  File operations
    // ─────────────────────────────────────────────────────────────────────────────

    static void DoNew()
    {
        g_scene.Clear();
        g_scene.currentPath.clear();
        g_scene.MarkClean();
        snprintf(g_pathBuf, sizeof(g_pathBuf), "scene.scn");
        g_selObj = g_selCol = -1;
        SetStatus("New scene created.", true);
    }

    static void DoLoad(const char* path)
    {
        if (g_scene.Load(path))
        {
            snprintf(g_pathBuf, sizeof(g_pathBuf), "%s", path);
            g_scene.MarkClean();
            g_selObj = g_scene.objects.empty() ? -1 : 0;
            g_selCol = -1;
            char msg[288];
            snprintf(msg, sizeof(msg), "Loaded %zu object(s) from '%s'.",
                g_scene.objects.size(), path);
            SetStatus(msg, true);
        }
        else
        {
            char msg[288];
            snprintf(msg, sizeof(msg), "Failed to load '%s'.", path);
            SetStatus(msg, false);
        }
    }

    static void DoSave(const char* path)
    {
        if (g_scene.Save(path))
        {
            snprintf(g_pathBuf, sizeof(g_pathBuf), "%s", path);
            g_scene.currentPath = path;
            g_scene.MarkClean();
            char msg[288];
            snprintf(msg, sizeof(msg), "Saved %zu object(s) to '%s'.",
                g_scene.objects.size(), path);
            SetStatus(msg, true);
        }
        else
        {
            char msg[288];
            snprintf(msg, sizeof(msg), "Failed to save '%s'.", path);
            SetStatus(msg, false);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Toolbar
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawToolbar()
    {
        bool modified = g_scene.IsModified();

        // ── New ──────────────────────────────────────────────────────────────────
        if (ImGui::Button("New"))
        {
            if (modified) g_showNewConfirm = true;
            else          DoNew();
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Create a new empty scene");

        ImGui::SameLine();

        // ── Load ─────────────────────────────────────────────────────────────────
        if (ImGui::Button("Load"))
        {
            if (modified)
            {
                snprintf(g_pendingLoadPath, sizeof(g_pendingLoadPath), "%s", g_pathBuf);
                g_showLoadConfirm = true;
            }
            else DoLoad(g_pathBuf);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Load scene from path in text box");

        ImGui::SameLine();

        // ── Save ─────────────────────────────────────────────────────────────────
        if (ImGui::Button("Save"))
        {
            if (g_scene.currentPath.empty())
                g_showSaveAsPopup = true;
            else
                DoSave(g_scene.currentPath.c_str());
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Save to current path  (Ctrl+S)");

        ImGui::SameLine();

        // ── Save As ───────────────────────────────────────────────────────────────
        if (ImGui::Button("Save As"))
            g_showSaveAsPopup = true;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Save scene to a new file");

        ImGui::SameLine(0, 20);

        // ── Path box ──────────────────────────────────────────────────────────────
        float remaining = ImGui::GetContentRegionAvail().x;
        ImGui::SetNextItemWidth(remaining > 10.f ? remaining : 200.f);
        ImGui::InputText("##scnpath", g_pathBuf, sizeof(g_pathBuf));

        // ── Modified indicator ────────────────────────────────────────────────────
        if (modified)
        {
            ImGui::SameLine();
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4{ 1.f, 0.7f, 0.2f, 1.f });
            ImGui::TextUnformatted("●");
            ImGui::PopStyleColor();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Unsaved changes");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Scene action buttons (Apply / Capture)
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawSceneActions()
    {
        ImGui::Spacing();
        ImGui::SeparatorText("Scene Actions");

        // Apply
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.18f, 0.44f, 0.18f, 1.f });
        if (ImGui::Button("Apply All to Scene"))
        {
            int spawned = 0;
            for (const auto& o : g_scene.objects)
            {
                SpawnObject(o); ++spawned;
            }
            char msg[128];
            snprintf(msg, sizeof(msg), "Spawned %d object(s) into scene.", spawned);
            SetStatus(msg, true);
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Push all pending objects into the running simulation");

        ImGui::SameLine();

        // Capture
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.18f, 0.24f, 0.48f, 1.f });
        if (ImGui::Button("Capture Live Scene"))
        {
            g_scene.Clear();
            for (const auto& obj : sceneModels)
            {
                const ModelInstance& inst = obj.instance;
                SceneObject so;
                so.name = obj.name;
                so.parent = "none";
                so.modelPath = "Core/Resources/3dmodels/" + obj.name + ".obj";
                so.texturePath = "none";
                so.x = inst.position.x; so.y = inst.position.y; so.z = inst.position.z;
                so.rx = inst.rotation.x; so.ry = inst.rotation.y; so.rz = inst.rotation.z;
                so.sx = inst.scale.x;    so.sy = inst.scale.y;    so.sz = inst.scale.z;
                so.isStatic = false;
                so.weight = 0.f;
                g_scene.objects.push_back(so);
            }
            g_selObj = g_scene.objects.empty() ? -1 : 0;
            g_selCol = -1;
            g_scene.MarkDirty();
            char msg[128];
            snprintf(msg, sizeof(msg), "Captured %zu live object(s).",
                g_scene.objects.size());
            SetStatus(msg, true);
        }
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Import the live simulation state into the editor");
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Object tree panel (left side)
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawObjectTree(float width)
    {
        ImGui::BeginGroup();

        ImGui::SeparatorText("Objects");

        // Add / Remove buttons
        if (ImGui::SmallButton("+ Object"))
        {
            SceneObject o;
            o.name = "Object_" + std::to_string(g_scene.objects.size());
            o.modelPath = "Core/Resources/3dmodels/cube.obj";
            g_scene.objects.push_back(o);
            g_selObj = (int)g_scene.objects.size() - 1;
            g_selCol = -1;
            g_scene.MarkDirty();
            SetStatus("Object added.", true);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Dup.") && g_selObj >= 0)
        {
            SceneObject copy = g_scene.objects[g_selObj];
            copy.name += "_copy";
            g_scene.objects.insert(g_scene.objects.begin() + g_selObj + 1, copy);
            g_selObj += 1;
            g_scene.MarkDirty();
            SetStatus("Object duplicated.", true);
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.5f, 0.1f, 0.1f, 1.f });
        if (ImGui::SmallButton("Del.") && g_selObj >= 0)
        {
            g_scene.objects.erase(g_scene.objects.begin() + g_selObj);
            g_scene.MarkDirty();
            ClampSel();
            SetStatus("Object deleted.", true);
        }
        ImGui::PopStyleColor();

        ImGui::Separator();

        // Scrollable list
        ImGui::BeginChild("##objtree", ImVec2(width, 0), false,
            ImGuiWindowFlags_HorizontalScrollbar);

        // Group objects by parent for a simple two-level tree
        // Root objects (parent == "none") shown first; children indented
        std::vector<int> roots, displayed;

        for (int i = 0; i < (int)g_scene.objects.size(); ++i)
            if (g_scene.objects[i].parent == "none")
                roots.push_back(i);

        // Draw root then children recursively (one level deep for simplicity)
        std::function<void(int, int)> DrawNode = [&](int idx, int depth)
            {
                displayed.push_back(idx);
                const auto& obj = g_scene.objects[idx];

                // Indent
                if (depth > 0)
                    ImGui::Indent(14.f * (float)depth);

                // Drag source for reordering
                bool selected = (g_selObj == idx);

                ImGui::PushID(idx);

                char nodeLabel[128];
                snprintf(nodeLabel, sizeof(nodeLabel),
                    "%s%s%s",
                    (obj.colliders.empty() ? "" : "▸ "),
                    obj.name.c_str(),
                    (obj.isStatic ? "  [S]" : ""));

                // Selectable row
                if (ImGui::Selectable(nodeLabel, selected,
                    ImGuiSelectableFlags_AllowDoubleClick,
                    ImVec2(0.f, 0.f)))
                {
                    g_selObj = idx;
                    g_selCol = -1;
                }

                // Drag-reorder (same level only)
                if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID))
                {
                    g_dragSrc = idx;
                    ImGui::SetDragDropPayload("OBJ_IDX", &idx, sizeof(int));
                    ImGui::Text("Move: %s", obj.name.c_str());
                    ImGui::EndDragDropSource();
                }
                if (ImGui::BeginDragDropTarget())
                {
                    if (const ImGuiPayload* pl =
                        ImGui::AcceptDragDropPayload("OBJ_IDX"))
                    {
                        int src = *(const int*)pl->Data;
                        if (src != idx)
                        {
                            SceneObject moved = g_scene.objects[src];
                            g_scene.objects.erase(g_scene.objects.begin() + src);
                            int dst = idx < src ? idx : idx - 1;
                            g_scene.objects.insert(
                                g_scene.objects.begin() + dst, moved);
                            g_selObj = dst;
                            g_scene.MarkDirty();
                        }
                    }
                    ImGui::EndDragDropTarget();
                }

                // Colliders as sub-items (when this object is selected)
                if (selected && !obj.colliders.empty())
                {
                    for (int ci = 0; ci < (int)obj.colliders.size(); ++ci)
                    {
                        ImGui::Indent(18.f);
                        char cLabel[128];
                        snprintf(cLabel, sizeof(cLabel), "⬡ %s [%s]%s",
                            obj.colliders[ci].name.c_str(),
                            ColliderShapeName(obj.colliders[ci].shape),
                            obj.colliders[ci].isTrigger ? " *T*" : "");

                        bool cSel = (g_selCol == ci);
                        if (ImGui::Selectable(cLabel, cSel,
                            0, ImVec2(0.f, 0.f)))
                        {
                            g_selCol = ci;
                        }
                        ImGui::Unindent(18.f);
                    }
                }

                ImGui::PopID();

                if (depth > 0)
                    ImGui::Unindent(14.f * (float)depth);

                // Draw children
                for (int j = 0; j < (int)g_scene.objects.size(); ++j)
                    if (j != idx && g_scene.objects[j].parent == obj.name)
                        DrawNode(j, depth + 1);
            };

        for (int r : roots)
            DrawNode(r, 0);

        // Any object not yet displayed (orphan / unknown parent) shown at root
        for (int i = 0; i < (int)g_scene.objects.size(); ++i)
        {
            if (std::find(displayed.begin(), displayed.end(), i) == displayed.end())
                DrawNode(i, 0);
        }

        ImGui::EndChild(); // ##objtree
        ImGui::EndGroup();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Collider inspector (inside the inspector panel)
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawColliderInspector(SceneCollider& c)
    {
        ImGui::SeparatorText("Collider");

        // Name
        char nameBuf[64];
        snprintf(nameBuf, sizeof(nameBuf), "%s", c.name.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("Name##cn", nameBuf, sizeof(nameBuf)))
            c.name = nameBuf;

        // Shape selector
        static const char* shapeNames[] = {
            "Box", "Sphere", "Capsule", "ConvexHull", "TriangleMesh"
        };
        int shapeIdx = (int)c.shape;
        ImGui::SetNextItemWidth(-1);
        if (ImGui::Combo("Shape##cs", &shapeIdx, shapeNames, 5))
            c.shape = (ColliderShape)shapeIdx;

        ImGui::Spacing();

        // Offset transform
        ImGui::SeparatorText("Offset Transform");

        float pos[3] = { c.ox, c.oy, c.oz };
        ImGui::Text("Position"); ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##cop", pos, 0.01f, -1e4f, 1e4f, "%.3f"))
        {
            c.ox = pos[0]; c.oy = pos[1]; c.oz = pos[2];
        }

        float rot[3] = { c.rx, c.ry, c.rz };
        ImGui::Text("Rotation"); ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##cor", rot, 0.1f, -360.f, 360.f, "%.2f"))
        {
            c.rx = rot[0]; c.ry = rot[1]; c.rz = rot[2];
        }

        // Shape parameters
        ImGui::Spacing();
        ImGui::SeparatorText("Shape Parameters");
        switch (c.shape)
        {
        case ColliderShape::Box:
            ImGui::Text("Half-extents");
            ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##cbs", &c.sx, 0.005f, 0.001f, 100.f, "%.3f");
            break;
        case ColliderShape::Sphere:
            ImGui::SetNextItemWidth(120);
            ImGui::DragFloat("Radius##csr", &c.sx, 0.005f, 0.001f, 100.f, "%.3f");
            break;
        case ColliderShape::Capsule:
            ImGui::SetNextItemWidth(120);
            ImGui::DragFloat("Radius##ccr", &c.sx, 0.005f, 0.001f, 100.f, "%.3f");
            ImGui::SetNextItemWidth(120);
            ImGui::DragFloat("Half-height##cch", &c.sy, 0.005f, 0.001f, 100.f, "%.3f");
            break;
        case ColliderShape::ConvexHull:
        case ColliderShape::TriangleMesh:
            ImGui::TextDisabled("Built from parent mesh vertices.");
            break;
        }

        // Material overrides
        ImGui::Spacing();
        ImGui::SeparatorText("Material Overrides  (-1 = inherit)");
        ImGui::SetNextItemWidth(110);
        ImGui::DragFloat("Friction##cf", &c.friction, 0.01f, -1.f, 1.f, "%.2f");
        ImGui::SetNextItemWidth(110);
        ImGui::DragFloat("Restitution##cr", &c.restitution, 0.01f, -1.f, 1.f, "%.2f");

        // Flags
        ImGui::Spacing();
        ImGui::Checkbox("Trigger (sensor – no physics response)", &c.isTrigger);
        ImGui::Checkbox("Enabled##ce", &c.enabled);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Object inspector panel (right side)
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawInspector()
    {
        if (g_selObj < 0 || g_selObj >= (int)g_scene.objects.size())
        {
            ImGui::TextDisabled("No object selected.");
            return;
        }

        SceneObject& o = g_scene.objects[g_selObj];
        bool changed = false;

        // ── Identity ─────────────────────────────────────────────────────────────
        ImGui::SeparatorText("Identity");

        char nameBuf[64];
        snprintf(nameBuf, sizeof(nameBuf), "%s", o.name.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("Name##on", nameBuf, sizeof(nameBuf)))
        {
            o.name = nameBuf; changed = true;
        }

        // Parent selector (combo)
        auto parents = ValidParents(g_selObj);
        int parentIdx = 0;
        for (int i = 0; i < (int)parents.size(); ++i)
            if (parents[i] == o.parent) { parentIdx = i; break; }

        ImGui::SetNextItemWidth(-1);
        if (ImGui::BeginCombo("Parent##op", parents[parentIdx].c_str()))
        {
            for (int i = 0; i < (int)parents.size(); ++i)
            {
                bool sel = (i == parentIdx);
                if (ImGui::Selectable(parents[i].c_str(), sel))
                {
                    o.parent = parents[i]; changed = true;
                }
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        // ── Assets ───────────────────────────────────────────────────────────────
        ImGui::Spacing();
        ImGui::SeparatorText("Assets");

        char mpBuf[256];
        snprintf(mpBuf, sizeof(mpBuf), "%s", o.modelPath.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("Model##om", mpBuf, sizeof(mpBuf)))
        {
            o.modelPath = mpBuf; changed = true;
        }

        char tpBuf[256];
        snprintf(tpBuf, sizeof(tpBuf), "%s", o.texturePath.c_str());
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("Texture##ot", tpBuf, sizeof(tpBuf)))
        {
            o.texturePath = tpBuf; changed = true;
        }

        // ── Transform ────────────────────────────────────────────────────────────
        ImGui::Spacing();
        ImGui::SeparatorText("Transform");

        float pos[3] = { o.x,  o.y,  o.z };
        ImGui::Text("Position"); ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##op", pos, 0.01f, -1e4f, 1e4f, "%.3f"))
        {
            o.x = pos[0]; o.y = pos[1]; o.z = pos[2]; changed = true;
        }

        float rot[3] = { o.rx, o.ry, o.rz };
        ImGui::Text("Rotation"); ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##or", rot, 0.1f, -360.f, 360.f, "%.2f"))
        {
            o.rx = rot[0]; o.ry = rot[1]; o.rz = rot[2]; changed = true;
        }

        float scl[3] = { o.sx, o.sy, o.sz };
        ImGui::Text("Scale");    ImGui::SameLine(90);
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat3("##os", scl, 0.005f, 0.001f, 500.f, "%.3f"))
        {
            o.sx = scl[0]; o.sy = scl[1]; o.sz = scl[2]; changed = true;
        }

        // ── Physics ───────────────────────────────────────────────────────────────
        ImGui::Spacing();
        ImGui::SeparatorText("Physics");

        if (ImGui::Checkbox("Static##ost", &o.isStatic))           changed = true;
        ImGui::SetNextItemWidth(120);
        if (ImGui::DragFloat("Weight (kg)##ow", &o.weight,
            0.1f, 0.f, 1e6f, "%.1f"))             changed = true;
        if (o.weight <= 0.f)
            ImGui::SameLine(), ImGui::TextDisabled("(auto)");

        // Quick-apply single object
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.18f, 0.44f, 0.18f, 1.f });
        if (ImGui::Button("Spawn this object"))
        {
            SpawnObject(o);
            SetStatus(("Spawned: " + o.name).c_str(), true);
        }
        ImGui::PopStyleColor();

        // ── Colliders ─────────────────────────────────────────────────────────────
        ImGui::Spacing();
        ImGui::SeparatorText("Colliders");

        // Add / Remove / Duplicate
        if (ImGui::SmallButton("+ Add"))
        {
            SceneCollider c;
            c.name = "Collider_" + std::to_string(o.colliders.size());
            o.colliders.push_back(c);
            g_selCol = (int)o.colliders.size() - 1;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Dup.") && g_selCol >= 0
            && g_selCol < (int)o.colliders.size())
        {
            SceneCollider copy = o.colliders[g_selCol];
            copy.name += "_copy";
            o.colliders.insert(o.colliders.begin() + g_selCol + 1, copy);
            g_selCol++;
            changed = true;
        }
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.5f, 0.1f, 0.1f, 1.f });
        if (ImGui::SmallButton("Del.") && g_selCol >= 0
            && g_selCol < (int)o.colliders.size())
        {
            o.colliders.erase(o.colliders.begin() + g_selCol);
            if (g_selCol >= (int)o.colliders.size())
                g_selCol = (int)o.colliders.size() - 1;
            changed = true;
        }
        ImGui::PopStyleColor();

        // Collider list as a small box
        if (!o.colliders.empty())
        {
            float listH = std::min((int)o.colliders.size(), 4) * 20.f + 8.f;
            ImGui::BeginChild("##collist", ImVec2(-1, listH), true);
            for (int ci = 0; ci < (int)o.colliders.size(); ++ci)
            {
                char cLabel[128];
                snprintf(cLabel, sizeof(cLabel), " ⬡  %s  [%s]%s",
                    o.colliders[ci].name.c_str(),
                    ColliderShapeName(o.colliders[ci].shape),
                    o.colliders[ci].isTrigger ? "  T" : "");
                bool cSel = (g_selCol == ci);
                if (ImGui::Selectable(cLabel, cSel))
                    g_selCol = ci;
            }
            ImGui::EndChild();
        }
        else
        {
            ImGui::TextDisabled("No colliders. Click '+ Add' to create one.");
        }

        // Collider detail inspector
        if (g_selCol >= 0 && g_selCol < (int)o.colliders.size())
        {
            ImGui::Spacing();
            if (DrawColliderInspector(o.colliders[g_selCol]), true)
                changed = true;
        }

        if (changed)
            g_scene.MarkDirty();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Modal dialogs
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawModals()
    {
        // ── New confirm ───────────────────────────────────────────────────────────
        if (g_showNewConfirm)
        {
            ImGui::OpenPopup("Unsaved Changes – New");
            g_showNewConfirm = false;
        }
        if (ImGui::BeginPopupModal("Unsaved Changes – New",
            nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("The scene has unsaved changes.\nDiscard and create a new scene?");
            ImGui::Separator();
            if (ImGui::Button("Discard & New", { 130, 0 }))
            {
                DoNew(); ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", { 80, 0 }))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        // ── Load confirm ──────────────────────────────────────────────────────────
        if (g_showLoadConfirm)
        {
            ImGui::OpenPopup("Unsaved Changes – Load");
            g_showLoadConfirm = false;
        }
        if (ImGui::BeginPopupModal("Unsaved Changes – Load",
            nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Unsaved changes will be lost.\nLoad '%s' anyway?",
                g_pendingLoadPath);
            ImGui::Separator();
            if (ImGui::Button("Discard & Load", { 130, 0 }))
            {
                DoLoad(g_pendingLoadPath); ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", { 80, 0 }))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }

        // ── Save As ───────────────────────────────────────────────────────────────
        if (g_showSaveAsPopup)
        {
            ImGui::OpenPopup("Save As");
            g_showSaveAsPopup = false;
        }
        if (ImGui::BeginPopupModal("Save As",
            nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Save scene to:");
            ImGui::SetNextItemWidth(360);
            ImGui::InputText("##sapath", g_pathBuf, sizeof(g_pathBuf));
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4{ 0.18f, 0.44f, 0.18f, 1.f });
            if (ImGui::Button("Save", { 80, 0 }))
            {
                DoSave(g_pathBuf); ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor();
            ImGui::SameLine();
            if (ImGui::Button("Cancel", { 80, 0 }))
                ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    //  Status bar
    // ─────────────────────────────────────────────────────────────────────────────

    static void DrawStatusBar()
    {
        ImGui::Separator();
        if (g_statusTimer > 0.f)
        {
            ImGui::PushStyleColor(ImGuiCol_Text,
                g_statusOk ? ImVec4{ 0.30f,1.00f,0.40f,1.f }
            : ImVec4{ 1.00f,0.35f,0.35f,1.f });
            ImGui::TextUnformatted(g_status);
            ImGui::PopStyleColor();
            g_statusTimer -= ImGui::GetIO().DeltaTime;
        }
        else
        {
            // Show object/collider count in dim style
            ImGui::TextDisabled("%zu object(s)  |  sel: obj=%d  col=%d  |  %s",
                g_scene.objects.size(), g_selObj, g_selCol,
                g_scene.IsModified() ? "MODIFIED" : "saved");
        }
    }

} // namespace SceneEd

// ═════════════════════════════════════════════════════════════════════════════
//  Public entry point  (called from debugui.cpp)
// ═════════════════════════════════════════════════════════════════════════════
void DebugUI_RenderSceneEditor()
{
    using namespace SceneEd;

    // ── Toolbar (file ops + path) ─────────────────────────────────────────────
    DrawToolbar();
    ImGui::Spacing();

    // ── Scene Actions (Apply / Capture) ──────────────────────────────────────
    DrawSceneActions();
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // ── Main two-panel layout ─────────────────────────────────────────────────
    float fullW = ImGui::GetContentRegionAvail().x;
    float statusH = ImGui::GetTextLineHeightWithSpacing() + 6.f;
    float treeW = std::max(160.f, fullW * 0.30f);
    float inspectW = fullW - treeW - ImGui::GetStyle().ItemSpacing.x;
    float panelH = ImGui::GetContentRegionAvail().y - statusH - 8.f;

    // Left: object tree
    ImGui::BeginChild("##treePanel", ImVec2(treeW, panelH), true);
    DrawObjectTree(treeW - 6.f);
    ImGui::EndChild();

    ImGui::SameLine();

    // Right: inspector
    ImGui::BeginChild("##inspectPanel", ImVec2(inspectW, panelH), true,
        ImGuiWindowFlags_HorizontalScrollbar);
    DrawInspector();
    ImGui::EndChild();

    // ── Modals ────────────────────────────────────────────────────────────────
    DrawModals();

    // ── Status bar ────────────────────────────────────────────────────────────
    DrawStatusBar();
}
