import { getSimpleRouteJson } from "solver-utils"
import { test, expect } from "bun:test"
import { circuitJsonToPcbSvg } from "circuit-to-svg"
import { Circuit } from "@tscircuit/core"
import { getDebugSvg } from "../../../infinite-grid-ijump-astar/tests/fixtures/get-debug-svg"
import { MultilayerIjump } from "../../MultilayerIjump"
import { ObstacleList3d } from "../../ObstacleList3d"

const OneByOnePad = (props: {
  name: string
  pcbX?: number
  pcbY?: number
  layer: string
}) => (
  <chip name={props.name} pcbX={props.pcbX} pcbY={props.pcbY}>
    <footprint>
      <smtpad
        pcbX={0}
        pcbY={0}
        shape="rect"
        width="1mm"
        height="1mm"
        layer={props.layer as any}
        portHints={["pin1"]}
      />
    </footprint>
  </chip>
)

// https://github.com/tscircuit/autorouting/issues/92
// When a node hits an obstacle and a sideways direction is completely clear
// (wallDistance === Infinity), the solver used to push a "goal-snap" candidate
// with travelDistance = distAlongDir(node, goal). distAlongDir is unsigned, so
// when the goal was actually BEHIND that direction the candidate teleported
// the trace away from the goal — a "wild jump". These tests check that the
// bogus candidate is no longer generated and that the solved route stays sane.

test("repro3: no wrong-direction goal-snap candidates when a wall is hit", () => {
  const input: any = {
    layerCount: 2,
    minTraceWidth: 0.15,
    obstacles: [
      // wall blocking +x ahead of the node
      {
        type: "rect",
        layers: ["top", "bottom"],
        center: { x: 1, y: 0 },
        width: 1,
        height: 2,
        connectedTo: [],
      },
    ],
    connections: [
      {
        name: "n1",
        pointsToConnect: [
          { x: -1, y: 0, layer: "top" },
          { x: 3, y: -1.7, layer: "bottom" },
        ],
      },
    ],
    bounds: { minX: -5, maxX: 5, minY: -5, maxY: 5 },
  }

  const autorouter = new MultilayerIjump({ input, VIA_COST: 1 })
  autorouter.startNode = { x: -1, y: 0, l: 0 } as any
  autorouter.goalPoint = { x: 3, y: -1.7, l: 1 } as any
  autorouter.obstacles = autorouter.createObstacleList({
    connection: input.connections[0],
    obstaclesFromTraces: [],
  }) as any

  const wall = autorouter.obstacles.getObstaclesOverlappingRegion({
    minX: 0,
    minY: -1,
    maxX: 2,
    maxY: 1,
    l: 0,
  })[0]
  const parent = { x: -1, y: 0, l: 0, g: 0 } as any
  const node = {
    x: 0.35,
    y: 0,
    l: 0,
    g: 1.35,
    obstacleHit: wall,
    parent,
  } as any

  const neighbors = autorouter.getNeighbors(node)

  // The goal is at y = -2 while the open direction is +y. The buggy code
  // generated a candidate at y = +2 (the unsigned projection of the goal onto
  // the wrong direction). No candidate may land on the "mirror" of the goal
  // across the node along an open direction.
  const wild = neighbors.filter(
    (n: any) =>
      (n.y !== node.y &&
        Math.abs(n.y - (2 * node.y - autorouter.goalPoint.y)) < 0.01) ||
      (n.x !== node.x &&
        Math.abs(n.x - (2 * node.x - autorouter.goalPoint.x)) < 0.01),
  )
  expect(wild).toHaveLength(0)
})

test("repro3: wild trace jump scene routes cleanly", () => {
  const circuit = new Circuit()

  circuit.add(
    <board width="10mm" height="8mm" routingDisabled>
      <OneByOnePad name="U1" pcbX={-1} pcbY={0} layer="top" />
      <OneByOnePad name="U2" pcbX={3} pcbY={-2} layer="bottom" />
      <chip name="U_obstacle" pcbX={1} pcbY={0}>
        <footprint>
          <smtpad
            pcbX={0}
            pcbY={0}
            shape="rect"
            width="1mm"
            height="2mm"
            layer="top"
            portHints={["pin1"]}
          />
        </footprint>
      </chip>
      <trace from=".U1 > .pin1" to=".U2 > .pin1" />
    </board>,
  )

  const circuitJson = circuit.getCircuitJson()
  const input = getSimpleRouteJson(circuitJson as any, { layerCount: 2 })

  const autorouter = new MultilayerIjump({ input, VIA_COST: 1, debug: true })
  const solution = autorouter.solveAndMapToTraces()

  expect(
    circuitJsonToPcbSvg(circuitJson.concat(solution as any) as any),
  ).toMatchSvgSnapshot(import.meta.path)
})
