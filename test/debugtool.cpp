//-----------------------------------------------------------------------------
// Our entry point for exposing various internal mechanisms.
//
// Copyright 2017 whitequark
//-----------------------------------------------------------------------------

#include "solvespace.h"
#include "expr.h"
#include "platform/platform.h"
#ifdef HAVE_OPENCASCADE
#include "occ/solidmodel.h"
#endif

using namespace SolveSpace;

static const char *SolveResultName(SolveResult how) {
    switch(how) {
        case SolveResult::OKAY:                     return "OKAY";
        case SolveResult::DIDNT_CONVERGE:           return "DIDNT_CONVERGE";
        case SolveResult::REDUNDANT_OKAY:           return "REDUNDANT_OKAY";
        case SolveResult::REDUNDANT_DIDNT_CONVERGE: return "REDUNDANT_DIDNT_CONVERGE";
        case SolveResult::TOO_MANY_UNKNOWNS:        return "TOO_MANY_UNKNOWNS";
    }
    return "?";
}

// Load a file, regenerate it, and report how every group solved. Useful to
// reproduce solver failures without a GUI.
static int CmdSolve(const std::string &filename) {
    SS.Init();
    SS.showToolbar = false;
    SS.checkClosedContour = false;

    // Absolute, or a path linked from the file resolves against nothing.
    Platform::Path path = Platform::Path::From(filename).Expand(/*fromCurrentDirectory=*/true);
    if(!SS.LoadFromFile(path)) {
        fprintf(stderr, "cannot load: %s\n", filename.c_str());
        return 1;
    }
    SS.AfterNewFile();

    int failed = 0;
    for(Group &g : SK.group) {
        bool ok = g.IsSolvedOkay();
        if(!ok) failed++;
        fprintf(stderr, "group %08x %-24s %-24s dof=%d%s\n", g.h.v,
                g.DescriptionString().c_str(), SolveResultName(g.solved.how),
                g.solved.dof, ok ? "" : "  FAILED");
        for(int i = 0; i < g.solved.remove.n; i++) {
            Constraint *c = SK.constraint.FindByIdNoOops(g.solved.remove[i]);
            if(!c) continue;
            fprintf(stderr, "    bad constraint %08x %s\n", c->h.v,
                    c->DescriptionString().c_str());
        }
    }
    fprintf(stderr, "%s: %d group(s) failed to solve\n", filename.c_str(), failed);
    return failed == 0 ? 0 : 1;
}

// Load a file, regenerate it, and write it back out. This is what happens when
// a part that other files link to is opened, edited and saved again.
static int CmdResave(const std::string &filename) {
    SS.Init();
    SS.showToolbar = false;
    SS.checkClosedContour = false;

    Platform::Path path = Platform::Path::From(filename).Expand(/*fromCurrentDirectory=*/true);
    if(!SS.LoadFromFile(path)) {
        fprintf(stderr, "cannot load: %s\n", filename.c_str());
        return 1;
    }
    SS.AfterNewFile();
    if(!SS.SaveToFile(path)) {
        fprintf(stderr, "cannot save: %s\n", filename.c_str());
        return 1;
    }
    return 0;
}

// Write the last group's OpenCASCADE shape out as a BREP, which is the only
// way to get at what the program actually meshes rather than a copy of it.
static int CmdBrep(const std::string &filename, const std::string &out, int groupIndex) {
#ifdef HAVE_OPENCASCADE
    SS.Init();
    SS.showToolbar = false;
    SS.checkClosedContour = false;

    Platform::Path path = Platform::Path::From(filename).Expand(/*fromCurrentDirectory=*/true);
    if(!SS.LoadFromFile(path)) {
        fprintf(stderr, "cannot load: %s\n", filename.c_str());
        return 1;
    }
    SS.AfterNewFile();

    Group *g = SK.GetGroup(SS.GW.activeGroup);
    if(groupIndex > 0 && groupIndex <= SK.groupOrder.n) {
        g = SK.GetGroup(SK.groupOrder[groupIndex - 1]);
    }
    if(!g->runningSolidModel || g->runningSolidModel->IsEmpty()) {
        fprintf(stderr, "group %s has no OpenCASCADE shape\n",
                g->DescriptionString().c_str());
        return 1;
    }
    if(!g->runningSolidModel->ExportBREP(Platform::Path::From(out))) {
        fprintf(stderr, "cannot write: %s\n", out.c_str());
        return 1;
    }
    fprintf(stderr, "wrote %s from group %s\n", out.c_str(),
            g->DescriptionString().c_str());
    return 0;
#else
    fprintf(stderr, "built without OpenCASCADE\n");
    return 1;
#endif
}

int main(int argc, char **argv) {
    std::vector<std::string> args = Platform::InitCli(argc, argv);

    if(args.size() == 3 && args[1] == "solve") {
        return CmdSolve(args[2]);
    } else if(args.size() == 3 && args[1] == "resave") {
        return CmdResave(args[2]);
    } else if((args.size() == 4 || args.size() == 5) && args[1] == "brep") {
        return CmdBrep(args[2], args[3], args.size() == 5 ? atoi(args[4].c_str()) : 0);
    } else if(args.size() == 3 && args[1] == "expr") {
        std::string expr = args[2], err;
        Expr *e = Expr::Parse(expr.c_str(), &err);
        if(e == NULL) {
            fprintf(stderr, "cannot parse: %s\n", err.c_str());
        } else {
            fprintf(stderr, "%g\n", e->Eval());
        }
        Platform::FreeAllTemporary();
    } else {
        fprintf(stderr, "Usage: %s <command> <options>\n", args[0].c_str());
//-----------------------------------------------------------------------------> 80 col */
        fprintf(stderr, R"(
Commands:
    expr [expr]
        Evaluate an expression.
    solve [file.slvs]
        Load a file and report how each group solved.
    resave [file.slvs]
        Load a file, regenerate it, and save it back.
    brep [file.slvs] [out.brep] [group]
        Write a group's OpenCASCADE shape to a BREP file, the last by default.
)");
    }

    return 0;
}
