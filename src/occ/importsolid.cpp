//-----------------------------------------------------------------------------
// Reading a STEP, BREP or IGES file into a group, and the bounding box
// entities that are the only thing a sketch can be constrained to on one.
//
// Split out of groupmesh.cpp unchanged.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "occ/solidmodel.h"

#ifdef HAVE_OPENCASCADE

namespace SolveSpace {

// Magic group numbers for imported solid bounding box entities (similar to importmesh.cpp)
// These avoid conflicts with actual entity handles
static const uint32_t IMPORT_SOLID_POINT_GROUP = 462;
static const uint32_t IMPORT_SOLID_NORMAL_GROUP = 472;
static const uint32_t IMPORT_SOLID_LINE_GROUP = 493;

// Helper functions for creating bounding box entities (similar to importmesh.cpp)
static hEntity newBBoxPoint(EntityList *el, int *id, Vector p) {
    Entity en = {};
    en.type = Entity::Type::POINT_N_COPY;
    en.extraPoints = 0;
    en.timesApplied = 0;
    en.group.v = IMPORT_SOLID_POINT_GROUP;
    en.actPoint = p;
    en.construction = false;  // Match importmesh.cpp
    en.style.v = Style::DATUM;
    en.actVisible = true;
    en.forceHidden = false;

    en.h.v = *id + IMPORT_SOLID_POINT_GROUP * 65536;
    *id = *id + 1;
    el->Add(&en);
    return en.h;
}

static hEntity newBBoxNormal(EntityList *el, int *id, Quaternion normal, hEntity p) {
    Entity en = {};
    en.type = Entity::Type::NORMAL_N_COPY;
    en.extraPoints = 0;
    en.timesApplied = 0;
    en.group.v = IMPORT_SOLID_NORMAL_GROUP;
    en.actNormal = normal;
    en.construction = false;  // Match importmesh.cpp
    en.style.v = Style::NORMALS;
    en.point[0] = p;
    en.actVisible = true;
    en.forceHidden = false;

    *id = *id + 1;
    en.h.v = *id + IMPORT_SOLID_NORMAL_GROUP * 65536;
    el->Add(&en);
    return en.h;
}

static hEntity newBBoxLine(EntityList *el, int *id, hEntity p0, hEntity p1) {
    Entity en = {};
    en.type = Entity::Type::LINE_SEGMENT;
    en.point[0] = p0;
    en.point[1] = p1;
    en.extraPoints = 0;
    en.timesApplied = 0;
    en.group.v = IMPORT_SOLID_LINE_GROUP;
    en.construction = false;  // Not construction so always visible
    en.style.v = 0;  // Use default style (ACTIVE_GRP when active)
    en.actVisible = true;
    en.forceHidden = false;

    en.h.v = *id + IMPORT_SOLID_LINE_GROUP * 65536;
    *id = *id + 1;
    el->Add(&en);
    return en.h;
}

// Create bounding box reference entities for an imported solid
static void CreateBoundingBoxEntities(EntityList *el, Vector minPt, Vector maxPt) {
    el->Clear();
    int id = 1;

    // Calculate center point for origin
    Vector center = minPt.Plus(maxPt).ScaledBy(0.5);

    // Add origin point at center and axis normals
    hEntity origin = newBBoxPoint(el, &id, center);
    newBBoxNormal(el, &id, Quaternion::From(Vector::From(1, 0, 0), Vector::From(0, 1, 0)), origin);
    newBBoxNormal(el, &id, Quaternion::From(Vector::From(0, 1, 0), Vector::From(0, 0, 1)), origin);
    newBBoxNormal(el, &id, Quaternion::From(Vector::From(0, 0, 1), Vector::From(1, 0, 0)), origin);

    // Create 8 corner points of bounding box
    hEntity p[8];
    p[0] = newBBoxPoint(el, &id, Vector::From(minPt.x, minPt.y, minPt.z));
    p[1] = newBBoxPoint(el, &id, Vector::From(maxPt.x, minPt.y, minPt.z));
    p[2] = newBBoxPoint(el, &id, Vector::From(minPt.x, maxPt.y, minPt.z));
    p[3] = newBBoxPoint(el, &id, Vector::From(maxPt.x, maxPt.y, minPt.z));
    p[4] = newBBoxPoint(el, &id, Vector::From(minPt.x, minPt.y, maxPt.z));
    p[5] = newBBoxPoint(el, &id, Vector::From(maxPt.x, minPt.y, maxPt.z));
    p[6] = newBBoxPoint(el, &id, Vector::From(minPt.x, maxPt.y, maxPt.z));
    p[7] = newBBoxPoint(el, &id, Vector::From(maxPt.x, maxPt.y, maxPt.z));

    // Create 12 edges of bounding box (4 bottom, 4 top, 4 vertical)
    // Bottom face
    newBBoxLine(el, &id, p[0], p[1]);
    newBBoxLine(el, &id, p[0], p[2]);
    newBBoxLine(el, &id, p[3], p[1]);
    newBBoxLine(el, &id, p[3], p[2]);

    // Top face
    newBBoxLine(el, &id, p[4], p[5]);
    newBBoxLine(el, &id, p[4], p[6]);
    newBBoxLine(el, &id, p[7], p[5]);
    newBBoxLine(el, &id, p[7], p[6]);

    // Vertical edges
    newBBoxLine(el, &id, p[0], p[4]);
    newBBoxLine(el, &id, p[1], p[5]);
    newBBoxLine(el, &id, p[2], p[6]);
    newBBoxLine(el, &id, p[3], p[7]);
}

void Group::PopulateBoundingBoxEntities() {
    if(type != Type::IMPORT_SOLID) return;
    if(linkFile.IsEmpty()) return;
    if(impEntity.n != 0) return;  // Already populated

    Vector minPt = {}, maxPt = {};
    if(SolidModelOcc::GetCachedBoundingBox(linkFile, &minPt, &maxPt)) {
        CreateBoundingBoxEntities(&impEntity, minPt, maxPt);
    }
}

// Read the file and make the bounding box entities. Returns true when the
// group is finished and GenerateShellAndMesh has nothing more to do, which is
// how an assembled import has always behaved.
bool Group::GenerateImportedSolid(Group *srcg) {
    // For ASSEMBLE mode: skip loading shape into thisSolidModel entirely
    // We'll use the cached mesh directly in GenerateDisplayItems
    if(srcg->meshCombine == CombineAs::ASSEMBLE) {
        if(!linkFile.IsEmpty()) {
            // Check if already cached; if not, do synchronous import
            if(!SolidModelOcc::GetCachedMesh(linkFile)) {
                bool success = false;
                SolidModelOcc m = SolidModelOcc::ImportCached(linkFile, &success);
                if(!success) {
                    OccFailed("Could not import '%s'. %s", linkFile.raw.c_str(),
                              m.importError.c_str());
                }
            }

            // Create bounding box entities from cache (if available)
            if(impEntity.n == 0) {
                Vector minPt = {}, maxPt = {};
                if(SolidModelOcc::GetCachedBoundingBox(linkFile, &minPt, &maxPt)) {
                    CreateBoundingBoxEntities(&impEntity, minPt, maxPt);
                }
            }
        }
        // Handled completely, as the early return here always meant.
        displayDirty = true;
        return true;
    }

    // Import solid geometry from STEP/BREP/IGES file into OCC solid model
    // Uses cached import to avoid re-reading file when group is recreated
    if(!linkFile.IsEmpty() && thisSolidModel && thisSolidModel->shape.IsNull()) {
        bool success = false;

        // Use cached import - avoids re-reading file and re-triangulating
        *thisSolidModel = SolidModelOcc::ImportCached(linkFile, &success);

        if(success) {
            // Create bounding box reference entities for constraining
            Vector minPt = {}, maxPt = {};
            thisSolidModel->GetBoundingBox(&minPt, &maxPt);
            CreateBoundingBoxEntities(&impEntity, minPt, maxPt);
        } else {
            OccFailed("Could not import '%s'. %s", linkFile.raw.c_str(),
                      thisSolidModel->importError.c_str());
        }
    }
    return false;
}

} // namespace SolveSpace

#endif // HAVE_OPENCASCADE
