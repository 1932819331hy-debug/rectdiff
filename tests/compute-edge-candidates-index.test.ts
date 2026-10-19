import { expect, test } from "bun:test"
import RBush from "rbush"
import type { XYRect } from "../lib/rectdiff-types"
import { computeEdgeCandidates3D } from "../lib/solvers/RectDiffSeedingSolver/computeEdgeCandidates3D"
import type { RTreeRect } from "../lib/types/capacity-mesh-types"
import { computeEdgeCandidates3DReference } from "./fixtures/computeEdgeCandidates3DReference"

type Layer = {
  obstacles?: XYRect[]
  hard?: XYRect[]
  soft?: XYRect[]
}

type Scenario = {
  name: string
  bounds: XYRect
  minSize: number
  layerCount: number
  layers: Array<Layer | undefined>
}

const rect = (x: number, y: number, width: number, height: number): XYRect => ({
  x,
  y,
  width,
  height,
})

function makeIndex(rects: XYRect[], z: number): RBush<RTreeRect> {
  return new RBush<RTreeRect>().load(
    rects.map((r) => ({
      ...r,
      minX: r.x,
      minY: r.y,
      maxX: r.x + r.width,
      maxY: r.y + r.height,
      zLayers: [z],
    })),
  )
}

function makeParams(scenario: Scenario) {
  return {
    bounds: scenario.bounds,
    minSize: scenario.minSize,
    layerCount: scenario.layerCount,
    obstacleIndexByLayer: scenario.layers.map((layer, z) =>
      layer ? makeIndex(layer.obstacles ?? [], z) : undefined,
    ),
    placedIndexByLayer: scenario.layers.map((layer, z) =>
      layer
        ? makeIndex([...(layer.hard ?? []), ...(layer.soft ?? [])], z)
        : undefined,
    ),
    hardPlacedByLayer: scenario.layers.map((layer) => layer?.hard ?? []),
  }
}

function makeRandomScenarios(): Scenario[] {
  let state = 0x109bcafe
  const random = () => {
    state = (Math.imul(state, 1664525) + 1013904223) >>> 0
    return state / 0x100000000
  }
  const integer = (max: number) => Math.floor(random() * max)
  const scenarios: Scenario[] = []

  for (let caseIndex = 0; caseIndex < 80; caseIndex++) {
    const layerCount = 1 + integer(6)
    const bounds = rect(
      -15 + random() * 20,
      -15 + random() * 20,
      2 + random() * 35,
      2 + random() * 35,
    )
    const layers: Array<Layer | undefined> = []
    // Some boards have truncated layer arrays as well as explicit missing layers.
    const suppliedLayerCount =
      caseIndex % 7 === 0 ? Math.max(0, layerCount - 1) : layerCount
    for (let z = 0; z < suppliedLayerCount; z++) {
      if (random() < 0.15) {
        layers.push(undefined)
        continue
      }
      const layer: Required<Layer> = { obstacles: [], hard: [], soft: [] }
      const count = integer(38)
      for (let i = 0; i < count; i++) {
        const jitter = (integer(3) - 1) * 0.00000049
        const r = rect(
          bounds.x + (random() * 1.6 - 0.3) * bounds.width + jitter,
          bounds.y + (random() * 1.6 - 0.3) * bounds.height - jitter,
          i % 9 === 0 ? 0.00000049 : random() * bounds.width * 0.6,
          i % 11 === 0 ? 0.00000151 : random() * bounds.height * 0.6,
        )
        const destination = [layer.obstacles, layer.hard, layer.soft][
          integer(3)
        ]!
        destination.push(r)
        if (i % 13 === 0) destination.push({ ...r })
      }
      layers.push(layer)
    }
    scenarios.push({
      name: `seeded board ${caseIndex}`,
      bounds,
      minSize: [0.00000001, 0.05, 0.2, 1, 3][caseIndex % 5]!,
      layerCount,
      layers,
    })
  }
  return scenarios
}

test("indexed edge candidates exactly match exhaustive analysis", () => {
  const duplicate = rect(4, 4, 2, 2)
  const scenarios: Scenario[] = [
    {
      name: "empty board with missing and empty layer indexes",
      bounds: rect(0, 0, 12, 10),
      minSize: 1,
      layerCount: 4,
      layers: [{}, undefined, {}],
    },
    {
      name: "obstacles, hard placements, soft placements, and duplicate geometry",
      bounds: rect(0, 0, 12, 10),
      minSize: 0.5,
      layerCount: 4,
      layers: [
        {
          obstacles: [duplicate, { ...duplicate }, rect(1, 1, 2, 3)],
          hard: [{ ...duplicate }, rect(8, 2, 1, 5)],
          soft: [rect(0, 0, 4, 10)],
        },
        { hard: [rect(0, 0, 4, 10)], soft: [rect(7, 0, 5, 10)] },
        { obstacles: [rect(0, 0, 4, 10)], soft: [rect(7, 0, 5, 10)] },
        { soft: [rect(0, 0, 12, 10)] },
      ],
    },
    {
      name: "fully occupied across obstacles and soft placements",
      bounds: rect(-4, -3, 12, 10),
      minSize: 0.5,
      layerCount: 3,
      layers: [
        { obstacles: [rect(-4, -3, 12, 10)] },
        { hard: [rect(-4, -3, 12, 10)] },
        { soft: [rect(-4, -3, 12, 10)] },
      ],
    },
    {
      name: "quantization boundaries and subprecision widths",
      bounds: rect(-0.00000049, 0.00000051, 12.00000049, 10.00000051),
      minSize: 0.2,
      layerCount: 2,
      layers: [
        {
          obstacles: [
            rect(2.00000049, 1.99999951, 0.99999951, 2.00000049),
            rect(4.00000051, 3.00000049, 0.00000049, 1.00000051),
            rect(6.99999949, 2.00000051, 0.00000051, 3.99999949),
            rect(0.03, 0.03, 1.00000049, 0.00000149),
          ],
          hard: [rect(9.00000051, 6.99999951, 1.00000051, 0.00000151)],
        },
        { soft: [rect(0, 0, 1, 10)] },
      ],
    },
    {
      name: "disjoint clipped intervals retain their original normalization",
      bounds: rect(0, 0, 12, 10),
      minSize: 1,
      layerCount: 2,
      layers: [
        {
          obstacles: [
            rect(4, 4, 2, 2),
            rect(3, -5, 1, 2),
            rect(3, 8, 1, 3),
            rect(-5, 3, 2, 1),
            rect(9, 3, 5, 1),
            rect(-8, -1, 2, 3),
            rect(14, -1, 2, 3),
            rect(-1, 14, 3, 2),
            rect(-1, -6, 3, 2),
          ],
        },
        {},
      ],
    },
    {
      name: "touching edges, narrow gaps, and zero-size geometry",
      bounds: rect(0, 0, 12, 10),
      minSize: 0.2,
      layerCount: 3,
      layers: [
        {
          obstacles: [
            rect(1, 1, 2, 2),
            rect(3, 1, 2, 2),
            rect(5.00000049, 1, 2, 2),
            rect(7.00000151, 1, 2, 2),
            rect(0, 0.03, 12, 0),
            rect(0.03, 0, 0, 10),
            rect(3, 3, 0, 0),
          ],
        },
        { hard: [rect(1, 3.03, 8, 0.01)] },
        {},
      ],
    },
    {
      name: "long thin rectangles and perpendicular offboard intervals",
      bounds: rect(-1000000, -2, 2000000, 4),
      minSize: 0.01,
      layerCount: 2,
      layers: [
        {
          obstacles: [
            rect(-2000000, -0.2, 4000000, 0.1),
            rect(-0.1, -1000000, 0.2, 2000000),
            rect(100, 1.5, 100000, 0.00000049),
            rect(-100000, -1.5, 100, 0.00000151),
          ],
          hard: [rect(-500000, -1, 1000000, 0.01)],
        },
        {},
      ],
    },
    {
      name: "EPS inset dominates a subprecision minimum size",
      bounds: rect(-1, -1, 2, 2),
      minSize: 0.00000001,
      layerCount: 1,
      layers: [
        {
          obstacles: [rect(0, -1, 0.00000049, 2), rect(-1, 0, 2, 0.00000051)],
        },
      ],
    },
    ...makeRandomScenarios(),
  ]

  for (const scenario of scenarios) {
    const params = makeParams(scenario)
    expect({
      scenario: scenario.name,
      candidates: computeEdgeCandidates3D(params),
    }).toEqual({
      scenario: scenario.name,
      candidates: computeEdgeCandidates3DReference(params),
    })
  }
})
