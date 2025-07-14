import {Line3, LonLat, Plane, Vec3} from "@openglobus/og";
import { wgs84 } from "@openglobus/og";

export function createParallelHatching(coordinates: [number, number, number?][][], step = 100, bearing = 0, offset = 50): LonLat[][] {
    const result:LonLat[][] = [];
    const resultVec3:Vec3[][] = [];

    const polygonCoordinates = coordinates[0];
    const polygonCoordinatesCartesian = convertLonLatToCartesian(polygonCoordinates);

    const startPoint = new LonLat(polygonCoordinates[0][0], polygonCoordinates[0][1]);

    const perpendicularAzimuth = (bearing + 90) % 360;
    const reversePerpendicularAzimuth = (perpendicularAzimuth + 180) % 360;
    
    let isFirstDirection = true;
    [perpendicularAzimuth, reversePerpendicularAzimuth].forEach(direction => {
        let isHatchingLineInsidePolygon = true;
        let i = isFirstDirection ? 0 : 1;
        isFirstDirection = false;

        while (isHatchingLineInsidePolygon) {      
            
            const points = getHatchingLinePoints(startPoint, direction, step * i++, polygonCoordinatesCartesian);

            if (points.length < 2) {
                isHatchingLineInsidePolygon = false;
            } else {
                // This is not always correct (in Concave polygons)
                for (let j = 0; j < points.length; j += 2) {
                    resultVec3.push(points.slice(j, j + 2));
                }
            }
        }
    })
    
    for (let j = 0; j < resultVec3.length; j++) {
        const pointsPair = resultVec3[j];
        const [extendedPoint1, extendedPoint2] = extendLine(pointsPair[0], pointsPair[1], offset);

        const pointsLonLat:LonLat[] = [
             wgs84.cartesianToLonLat(extendedPoint1)  ,
             wgs84.cartesianToLonLat(extendedPoint2)  
        ];

        result.push(pointsLonLat)
    }
    

    return result;
}

function getHatchingLinePoints(startPoint: LonLat, azimuth: number, distance: number, polygonCoordinatesCartesian: Vec3[]) : Vec3[] {

    /// For the first iteration - The plane is passing through the start point
    if ( distance == 0) distance = 1; // This is a dummy value used for generating the vector.

    const { destination } = wgs84.direct(startPoint, azimuth, distance);
    const destinationCartesian = wgs84.lonLatToCartesian(destination);
    const startPointCartesian = wgs84.lonLatToCartesian(startPoint);

    const pointInPlane = distance == 0 ? startPointCartesian : destinationCartesian;

    const segmentVector = Vec3.sub(destinationCartesian, startPointCartesian);
    const verticalPlaneNormal = segmentVector.normalize();
    const verticalPlane = new Plane(pointInPlane, verticalPlaneNormal);
    
    const intersections: Vec3[] = [];
    for (let index = 0; index < polygonCoordinatesCartesian.length - 1; index++) {
        const edge = new Line3(polygonCoordinatesCartesian[index], polygonCoordinatesCartesian[index + 1]);
        
        let intersection = intersectPlaneLine(verticalPlane, edge);
        
        if (intersection) {
            intersections.push(intersection);
        }
    }

    return intersections;
}

function convertLonLatToCartesian(lonLatCoordinates:[number, number, number?][]): Vec3[] {
    const result:Vec3[] = [];

    for (let index = 0; index < lonLatCoordinates.length; index++) {
        const coord = lonLatCoordinates[index];

        const coordCartesian = wgs84.lonLatToCartesian(new LonLat(coord[0], coord[1]));
        result.push(coordCartesian);        
    }

    return result;
}

function intersectPlaneLine(
  plane:Plane,
  line: Line3
): Vec3 | null {

  const planePoint: Vec3 = plane.p;
  const planeNormal: Vec3 = plane.n;
  const {p0, p1} = line;

  const lineDir = Vec3.sub(p1, p0);

  const denom = planeNormal.dot(lineDir);

  if (Math.abs(denom) < 1e-8) {
    return null;
  }

  const t = Vec3.dot(planeNormal, Vec3.sub(planePoint, p0)) / denom;

  if (t >= 0 && t <= 1) {
      const intersection = new Vec3(
          p0.x + t * lineDir.x,
          p0.y + t * lineDir.y,
          p0.z + t * lineDir.z
      );

      return intersection;
  }

  return null;
}

function extendLine(
  pointA: Vec3,
  pointB: Vec3,
  extendLength: number
): [Vec3, Vec3] {
  
  const dir: Vec3 = new Vec3(
    pointB.x - pointA.x,
    pointB.y - pointA.y,
    pointB.z - pointA.z,
  );

  const length = Math.sqrt(dir.x ** 2 + dir.y ** 2 + dir.z ** 2);

  if (length === 0) {
    throw new Error("Points A and B are the same. Cannot extend line.");
  }

  const unitDir: Vec3 = new Vec3 (
    dir.x / length,
    dir.y / length,
    dir.z / length,
  );

  const extendedPointA: Vec3 = new Vec3( 
    pointA.x - unitDir.x * extendLength,
    pointA.y - unitDir.y * extendLength,
    pointA.z - unitDir.z * extendLength,
    );

  const extendedPointB: Vec3 = new Vec3 (
    pointB.x + unitDir.x * extendLength,
    pointB.y + unitDir.y * extendLength,
    pointB.z + unitDir.z * extendLength,
  );

  return [extendedPointA, extendedPointB];
}