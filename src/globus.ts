import './style.css';
import {Bing, control, Globe, GlobusRgbTerrain, OpenStreetMap} from '@openglobus/og';

let osm = new OpenStreetMap('osm');

let sat = new Bing('sat')

let globus = new Globe({
    target: "app",
    name: "Earth",
    terrain: new GlobusRgbTerrain(),
    layers: [osm, sat],
    resourcesSrc: "./node_modules/@openglobus/og/lib/res",
    fontsSrc: "./node_modules/@openglobus/og/lib/res/fonts"
});
globus.planet.addControl(new control.RulerSwitcher())
export const GLOBUS = globus
