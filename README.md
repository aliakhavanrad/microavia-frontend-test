# Parallel Line Generation Algorithm For a Polygon

Implement an algorithm that generates a set of parallel lines by a given polygon which will be built with given step, bearing angle related to North and offset.  
The visualization scene and drawing tools are already prepared — your task is to generate the correct set of line coordinates for the polygon.


## Input Parameters

- **`polygonCoordinates`**: An array of coordinates describing a single polygon (no holes) in KML format. The polygon is defined as an array of `[longitude, latitude]` pairs, with the first and last coordinates equal (closed polygon). For example:

  ```ts
  [
    [55.5487277195291, 24.35455341474017],
    [55.52483599504373, 24.18609028446061],
    [55.66944139410629, 24.16878452908807],
    [55.69352433077309, 24.33724790559794],
    [55.5487277195291, 24.35455341474017]
  ]
  ```

- **`step`** *(default = 100)*: Distance between adjacent parallel lines, in **meters**.

- **`bearing`** *(default = 0)*: Orientation of the lines, in **degrees from North** (0° = North, 90° = East, etc.).

- **`offset`** *(default = 50)*: External offset in **meters** from edge of the polygon(lines should start and end "offset" meters outside the polygon)

---

## Requirements

- Implement the algorithm in:

  ```
  src/lines/parallelHatchingAlgorithm.ts
  ```

  as the following function:

  ```ts
  export function createParallelHatching(options: {
    polygonCoordinates: number[][],
    step?: number,
    bearing?: number,
    offset?: number
  }): LonLat[][] {
    //...
  }
  ```

- This function will be called from:

  ```
  src/main.ts
  ```

  on a double-click event on a polygon. Drawing lines function also placed there.

- Changing the `step`, `bearing`, or `offset` must update the drawn lines on the map.

- The result must visually match the behavior demonstrated in the reference [video](./assets/example.mp4) (equal and correct spacing between lines, correct orientation,  equal and correct distance from edge of polygon to start/end of line, full square of the polygon must be hatched by lines).

- Ensure performance optimizations for handling large polygons efficiently.

- Please share your implementation of algorithm using a fork of this [sandbox](https://codesandbox.io/p/devbox/jwhg43?file=%2Fpackage.json) or send it back in archive as full project.

---

## Available Tools

You can use the following utility classes from the [OpenGlobus library](https://github.com/openglobus/openglobus):

- [LonLat](https://github.com/openglobus/openglobus/blob/master/src/LonLat.ts) – Geographic coordinate representation.
- [Vec2](https://github.com/openglobus/openglobus/blob/master/src/math/Vec2.ts), [Vec3](https://github.com/openglobus/openglobus/blob/master/src/math/Vec3.ts) – Vector utilities.
- [Ellipsoid](https://github.com/openglobus/openglobus/blob/master/src/ellipsoid/Ellipsoid.ts) – For solving direct and inverse geodesic tasks, calculating distances, bearings, and destination points over the ellipsoid.
- You can also use any other libraries you find useful
---

## Optional

- Add a small UI widget to dynamically adjust:
  - Line spacing (`step`)
  - Line orientation (`bearing`)
  - Offset (`offset`)

---

## Output Validation

- Validate output visually using the built-in map ruler tool.
- Lines should be straight, correctly spaced, and confined to the polygons.

## Example Video

[Watch the video](./assets/example.mp4)

---

## Run scene
To run the scene locally, use the following command:

```bash
npm run dev
```