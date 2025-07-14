import {Vector} from "@openglobus/og";
import geojsonData from "./polygons.json";
import {Polygon} from "./polygon.ts";
import {geoJSONToGeometries} from "./utils.ts";


const geometries = geoJSONToGeometries(geojsonData as any);
const layer = new Vector('polygons', {});
geometries.forEach((g) => {
    layer.add(new Polygon({geometry: g}))
});

export const POLYGONS_LAYER = layer;