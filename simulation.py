from robot import Simulator, Point, Maze, Vec2i
maze = Maze((Vec2i(0, 0), Vec2i(8, 8)), 8)
sim = Simulator(maze.world_center(), [maze])
sim.animate(interval_ms=100)