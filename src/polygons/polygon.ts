import {Entity, Geometry, IEntityParams} from "@openglobus/og";

export class Polygon extends Entity {
    declare geometry: Geometry
    constructor(params: IEntityParams) {
        super(params);
    }

    get coordinates(): any {
        return <any>this.geometry._coordinates;
    }
}
