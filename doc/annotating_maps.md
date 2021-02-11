# Annotating Maps with Inkscape

`knowledge_representation` can store basic 2D geometry, including points, poses, and doors. The package provides a simple way to load these annotations from SVG data overlaid on a PGM map. This means annotations can be stored in the filesystem as a sidecar file to the map file that your robot is probably already using for navigation.

This document describes how to create annotations using Inkscape, a popular open-source SVG editor.

## Creating the annotation file

Open the PGM file you want to annotate with Inkscape, using File > Open.
Inkscape will ask if you want to embed or link to the image. Embedding will mean duplicating data, but Inkscape will apply compression that keeps the size manageable. Linking saves space, but Inkscape has a habit of using absolute paths to specify them, so if you don't fix this every time you commit your annotations, your collaborators will have to fix the link on their filesystem.

It's important that the bounds of the SVG exactly match the dimensions of the PGM. Make sure you leave the image positioned at (0, 0), or your annotations will be offset.

## Annotating

All annotations consist of some geometry and a text label naming that geometry.

### Points

Create a circle and position the center on the pixel coordinate you wish to mark. Create a text label and give the point a name. Group the circle and the label.

### Poses

Using the pen tool, draw a line. In the stroke style panel, set an arrow head marker on one end of the line pointing in the direction of the pose. Create a text label and give the pose a name. Group the line and the label.

### Regions

Using the pen tool, draw a closed region. Do not use curves, use only straight lines. Create a text label and give the region a name. Group the path and the label.

### Doors

Using the pen tool, draw a line segment. Make sure the path consists of exactly two points; a start point and an end point. Draw circles marking any associated approach points. Create a text label and give the region a name. Group the path, circles and label.

## Saving the annotation file

By default, Inkscape includes a large amount of metadata in the SVGs it outputs. To avoid creating unwieldy diffs in your version control system, select "Optimized SVG" from the save dialog file format selector. Ensure that "Collapse groups" is unchecked, then save the SVG.

## Example

An [example annotated map](https://github.com/utexas-bwi/knowledge_representation/blob/master/test/resources/map/map_inkscape.svg) is included in the repo.