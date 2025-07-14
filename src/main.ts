import './style.css'
import {GLOBUS} from "./globus.ts";
import {Polygon} from "./polygons/polygon.ts";
import {POLYGONS_LAYER} from "./polygons/layer.ts";
import {LINE_LAYER} from "./lines/layer.ts";
import {createParallelHatching} from "./lines/parallelHatchingAlgorithm.ts";
import {Line} from "./lines/line.ts";

GLOBUS.planet.addLayer(POLYGONS_LAYER)
GLOBUS.planet.addLayer(LINE_LAYER)

POLYGONS_LAYER.events.on('ldblclick', (e: any) => {
    try {
        if (e.pickingObject instanceof Polygon) {
            
            const polygon = e.pickingObject,
                coordinates = polygon.coordinates;

            LINE_LAYER.clear()
            createParallelHatching(coordinates).map((line) => {

                for (let i = 0; i < line.length; i += 2) {
                    const ll1 = line[i];
                    const ll2 = line[i + 1];
                    if (ll1 && ll2) {
                        LINE_LAYER.add(new Line([[ll1.lon, ll1.lat], [ll2.lon, ll2.lat]]))
                    }
                }

            })
        }
    } catch (e) {
        console.error(e)
    }
})

const entities = POLYGONS_LAYER.getEntities()
if (entities && entities.length > 0) {
    const lastPoly = entities.pop(),
        extent = lastPoly?.getExtent()

    extent && GLOBUS.planet.camera.flyExtent(extent)
}