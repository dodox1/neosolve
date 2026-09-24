//-----------------------------------------------------------------------------
// Routines to read a TrueType font as vector outlines, and generate them
// as entities, since they're always representable as either lines or
// quadratic Bezier curves.
//
// Copyright 2016 whitequark, Peter Barfuss.
//-----------------------------------------------------------------------------

#ifndef SOLVESPACE_TTF_H
#define SOLVESPACE_TTF_H

#include <string>

#include "dsc.h"
#include "platform/platform.h"

// We declare these in advance instead of simply using FT_Library
// (defined as typedef FT_LibraryRec_* FT_Library) because including
// freetype.h invokes indescribable horrors and we would like to avoid
// doing that every time we include solvespace.h.
struct FT_LibraryRec_;
struct FT_FaceRec_;

namespace SolveSpace {

class SBezierList;

class TtfFont {
public:
    Platform::Path  fontFile; // or resource path/name as res://<path>
    std::string     name;
    FT_FaceRec_    *fontFace;
    double          capHeight;
    // Why this font could not be used, or why its cap height is a guess.
    // Recorded rather than printed: scanning a font collection meets both of
    // these routinely, and one line per font buries everything else.
    std::string     loadError;
    bool            capHeightGuessed;

    void SetResourceID(const std::string &resource);
    bool IsResource() const;

    std::string FontFileBaseName() const;
    bool LoadFromFile(FT_LibraryRec_ *fontLibrary, bool keepOpen = false);
    bool LoadFromResource(FT_LibraryRec_ *fontLibrary, bool keepOpen = false);

    void PlotString(const std::string &str,
                    SBezierList *sbl, bool kerning, Vector origin, Vector u, Vector v);
    double AspectRatio(const std::string &str, bool kerning);

    bool ExtractTTFData(bool keepOpen);
};

class TtfFontList {
public:
    FT_LibraryRec_ *fontLibrary;
    bool            loaded;
    List<TtfFont>   l;

    TtfFontList();
    ~TtfFontList();

    void LoadAll();
    TtfFont *LoadFont(const std::string &font);

    void PlotString(const std::string &font, const std::string &str,
                    SBezierList *sbl, bool kerning, Vector origin, Vector u, Vector v);
    double AspectRatio(const std::string &font, const std::string &str, bool kerning);
};

} // namespace SolveSpace

#endif
