import {Entity, Geometry} from "@openglobus/og";

export class Line extends Entity {
    declare geometry: Geometry

    constructor(coordinates: [number, number][]) {
        super({
            geometry: {
                type: 'LINESTRING',
                style: {
                    lineWidth: 10,
                    lineColor: '#ff0000'
                },
                coordinates
            }
        });
    }

    get coordinates() {
        return this.geometry._coordinates;
    }
}
