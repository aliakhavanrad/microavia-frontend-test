import {GeometryTypeEnum, IGeometryParams, IGeometryStyle} from "@openglobus/og";
import {Feature, FeatureCollection} from "geojson";


function hexToRgba(cssHex: string, opacity = 1): string {
    const hex = cssHex.replace("#", "");
    const int = parseInt(hex, 16);
    const r = (int >> 16) & 255;
    const g = (int >> 8) & 255;
    const b = int & 255;
    return `rgba(${r},${g},${b},${opacity})`;
}

const geoJSON2GeometryType: Record<string, keyof typeof GeometryTypeEnum> = {
    Point: "POINT",
    LineString: "LINESTRING",
    Polygon: "POLYGON",
    MultiPolygon: "MULTIPOLYGON",
    MultiLineString: "MULTILINESTRING"
};

function featureToParams(f: Feature): IGeometryParams<'POLYGON'> | undefined {
    if (!f.geometry) return;
    if (f.geometry.type !== 'Polygon') return;

    const typeKey = geoJSON2GeometryType['Polygon'];

    if (!typeKey) return;
    const style: IGeometryStyle = {};
    if (f.properties) {
        const p: any = f.properties;
        if (p.fill) {
            style.fillColor = hexToRgba(p.fill, p.fill_opacity ?? 1);
        }
        if (p.stroke) {
            style.lineColor = hexToRgba(p.stroke, p.stroke_opacity ?? 1);
        }
        if (p.strokeWidth || p["stroke-width"]) {
            style.lineWidth = Number(p.strokeWidth ?? p["stroke-width"]);
        }
    }

    return  {
        type: 'POLYGON',
        coordinates: f.geometry.coordinates as any,
        style
    };
}


export function geoJSONToGeometries(
    geojson: FeatureCollection | Feature
): IGeometryParams<'POLYGON'>[] {
    const features =
        geojson.type === "FeatureCollection"
            ? geojson.features
            : [geojson];

    return features
        .map(featureToParams)
        .filter((f): f is IGeometryParams<'POLYGON'> => Boolean(f));

}
