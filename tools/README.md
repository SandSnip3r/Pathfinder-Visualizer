# Pathfinder Tools

1. [QGIS Converter](#translate_qgis_csv)

## QGIS Converter

[QGIS](https://qgis.org/) is a tool for working with geographic maps. This tool ([translate_qgis_csv.py](translate_qgis_csv.py)) will translate from an output format from QGIS to an input format accepted by Pathfinder. These instructions are written for QGIS LTS version 3.28.13.

My use case for QGIS was to map a real place in a world so that I could use Pathfinder to find the shortest path between various pairs of points. Lets walk through the steps to accomplish this using QGIS:

1. I added a Google Maps Satellite layer into QGIS, [using these instructions](https://www.geodose.com/2018/03/how-to-add-google-maps-layer-QGIS-3.html).
2. Create a new Shapefile layer.

![toolbar-create-layer](toolbar-create-layer.png)

3. Enter a path and filename for the Shapefile layer. Also, select Geometry type as LineString. Don't worry about any other fields.

![create-layer](create-layer.png)

4. Enable editing in the newly created Shapefile layer.

![toggle-editing](toggle-editing.png)

5. Click Add Line Feature

![add-line-feature](add-line-feature.png)

6. Click point by point to create a polyline. This polyline will be a boundary that Pathfinder will not be able to cross. The polyline does not need to create a closed polygon, but it may. Once you're done, right click and a box will open up for you to enter an ID for that polyline. We don't need any specific IDs, so feel free to enter whatever you want (including nothing).

![create-polyline](create-polyline.png)

If you hide the Google Maps Satellite background, you can easily see your newly created polyline:

![created-polyline](created-polyline.png)

7. Once you're done creating all of your polylines, it's time to export the data. First, run the Extract Vertices tool. Once it opens, just click Run. This will create a new layer.

![toolbar-extract-vertices](toolbar-extract-vertices.png)

8. On the new layer, right click it, go to Export, and click Save Features As...

![export-vertices-1](export-vertices-1.png)

9. Specify a path and filename for the final CSV file. Also set GEOMETRY as AS_XY.

![export-vertices-2](export-vertices-2.png)

10. Now, you should be able to run this translate_qgis_csv.py tool to convert from the exported vertices from QGIS into a file format which Pathfinder accepts. Make sure you specify the output path to have the .poly extension. Once that's done, in Pathfinder Visualization, click File->Open, and select the generated .poly file.