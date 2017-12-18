using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;

namespace ConsoleApplication2
{
    class Program
    {
        public static NodeArray NodesArray;

        public static void InitializeMap(StreamReader file)
        {
            var line = file.ReadLine();
            var height = 0;
            var widght = 0;
            line = file.ReadLine();
            if (line != null)
            {
                string[] splitLine = line.Split(' ');
                height = Convert.ToInt32(splitLine[1]);
            }
            line = file.ReadLine();
            if (line != null)
            {
                string[] splitLine = line.Split(' ');
                widght = Convert.ToInt32(splitLine[1]);
            }
            Node[,] nodesArray = new Node[height, widght];
            line = file.ReadLine();
            for (var i = 0; i < height; ++i)
            {
                line = file.ReadLine();
                for (var j = 0; j < widght; ++j)
                {
                    if (line != null)
                    {
                        var symbol = Convert.ToChar(line[j]);
                        nodesArray[i, j] = new Node(i, j, symbol);
                    }
                }
            }
            NodesArray = new NodeArray(height, widght, nodesArray);
        }

        public static List<Node> AStarBuTheBook(Node start, Node goal)
        {;
            if (start.IsObstacle || goal.IsObstacle)
            {
                Console.WriteLine("Can't run A*! Enter proper from and to, one of them is an obsttacle!");
                return null;
            }
            // The set of nodes already evaluated
            var closedSet = new List<Node>();

            // The set of currently discovered nodes that are not evaluated yet.
            // Initially, only the start node is known.
            var openSet = new List<Node> {start};

            // For each node, which node it can most efficiently be reached from.
            // If a node can be reached from many nodes, cameFrom will eventually contain the
            // most efficient previous step.
            var cameFrom = new List<Node>();

            while (openSet.Count != 0)
            {
                openSet = openSet.OrderBy(arg => arg.DistanceToGoal).ToList();
                var current = openSet[0];

                if (current.Position == goal.Position)
                {
                    return reconstructPath(cameFrom, current);
                }

                var neighbours = NodesArray.Neighbours(current);

                openSet.Remove(current);
                closedSet.Add(current);

                foreach (var neighbour in neighbours)
                {
                    if(closedSet.Exists(arg => arg.Position == neighbour.Position))
                        continue; // Ignore the neighbor which is already evaluated.
                    if(!openSet.Exists(arg => arg.Position == neighbour.Position))
                        openSet.Add(neighbour); // Discover a new node
                    var tentativeDistance = current.DistanceToStart + Node.MetricsAStar(current, neighbour);
                    if (tentativeDistance > neighbour.DistanceToStart) continue; // This is not a better path.

                    // This path is the best until now. Record it!
                    if (!cameFrom.Exists(arg => arg.Position == current.Position))
                        cameFrom.Add(current);
                    neighbour.DistanceToStart = tentativeDistance;
                    neighbour.DistanceToGoal = neighbour.DistanceToStart + Node.MetricsAStar(neighbour, goal);
                }
            }

            return null;
        }

        private static List<Node> reconstructPath(List<Node> cameFrom, Node goal)
        {
            var path = new List<Node>();
            path.Add(goal);
            foreach (var node in cameFrom)
            {
                path.Add(node);
            }
            path.Reverse();
            return path;
        }

        public static List<Node> AStar(Node from, Node to)
        {
            var finalPath = new List<Node>();
            if (from.IsObstacle || to.IsObstacle)
            {
                Console.WriteLine("Can't run A*! Enter proper from and to, one of them is an obsttacle!");
                return finalPath;
            }
            var current = new Node(from) {DistanceToGoal = Node.MetricsAStar(@from, to), Visited = NodeState.Processed};
            var observed = new List<Node> {current};

            while (current.Position != to.Position)
            {
                var query = NodesArray.Neighbours(current);
                query = query.Where(node => !node.IsObstacle).ToList();
                foreach (var node in query)
                {
                    if (!observed.Exists(arg => arg.Position == node.Position))
                    {
                        node.DistanceToGoal = Node.MetricsAStar(node, to);
                        node.Visited = NodeState.Discovered;
                        observed.Add(node);
                    }
                }
                observed = observed.OrderBy(arg => arg.Visited).ThenBy(arg => arg.DistanceToGoal).ToList();
                if (observed[0].Visited != NodeState.Processed)
                {
                    current = observed[0];
                    observed[0].Visited = NodeState.Processed;
                }
                else
                {
                    Console.WriteLine("No path was found");
                    return finalPath;
                }
            }
            observed = observed.Where(informer => informer.Visited == NodeState.Processed).ToList();

            finalPath.Add(observed[0]);
            for (var i = 0; i < observed.Count; ++i)
            {
                var maxIndex = i;
                for (var j = i + 1; j < observed.Count; ++j)
                {
                    if (StraightLine.OnOneLine(observed[i], observed[j], NodesArray)) maxIndex = j;
                }
                if (maxIndex != i)
                {
                    var points = StraightLine.FindMiddlePoints(observed[i], observed[maxIndex]);
                    points.RemoveAt(0);
                    foreach (var point in points)
                    {
                        var coordinatesX = Convert.ToInt32(point.X);
                        var coordinatesY = Convert.ToInt32(point.Y);
                        finalPath.Add(NodesArray.Array[coordinatesX, coordinatesY]);
                    }
                    i = maxIndex-1;
                }
            }
            finalPath.Reverse();
            return finalPath;
        }

        public static float PathLength(List<Node> path)
        {
            var length = 0f;
            for (var i = 0; i < path.Count - 1; ++i)
            {
                length = length + Vector2.Distance(path[i].Position, path[i + 1].Position);
            }
            return length;
        }

        static void Main()
        {
            var file = new System.IO.StreamReader("arena.map");
            InitializeMap(file);
            file.Close();

            var tasks = new System.IO.StreamReader("arena.map.scen");
            var line = tasks.ReadLine();
            var caseNumber = 0;
            var successCounter = 0;
            //Create directory to save visualization
            var directoryName = Directory.GetCurrentDirectory();
            directoryName = Path.GetFullPath(Path.Combine(directoryName, @"..\..\"));
            directoryName = Path.Combine(directoryName, "Paths");
            if (Directory.Exists(directoryName)) Directory.Delete(directoryName, true);
            System.IO.Directory.CreateDirectory(directoryName);
            while ((line = tasks.ReadLine()) != null)
            {
                // Get Case
                ++caseNumber;
                string[] splitLine = line.Split('\t');
                var from = NodesArray.Array[Convert.ToInt32(splitLine[4]), Convert.ToInt32(splitLine[5])];
                var to = NodesArray.Array[Convert.ToInt32(splitLine[6]), Convert.ToInt32(splitLine[7])];
                var optimalLength = Convert.ToDouble(splitLine[8], new CultureInfo("en-US"));

                //Find path using A*
                //var path = AStar(from, to);

                //Find path using A* by the book
                var path = AStarBuTheBook(from, to);
                var length = 0f;
                if (path != null)
                {
                    length = PathLength(path);
                    //Visualize path
                    var fileName = "Case" + Convert.ToString(caseNumber) + ".map";
                    var pathVisualizer = System.IO.File.Create(Path.Combine(directoryName, fileName));
                    //Save visualization to file
                    for (var i = 0; i < NodesArray.height; ++i)
                    {
                        for (var j = 0; j < NodesArray.widght; ++j)
                        {
                            if (
                                path.Exists(
                                    arg => Convert.ToInt32(arg.Position.X) == i && Convert.ToInt32(arg.Position.Y) == j
                                           && arg.Position != from.Position && arg.Position != to.Position))
                            {
                                pathVisualizer.WriteByte(Convert.ToByte('*'));
                            }
                            else if (NodesArray.Array[i, j].IsObstacle && NodesArray.Array[i, j] != from
                                     && NodesArray.Array[i, j] != to)
                            {
                                pathVisualizer.WriteByte(Convert.ToByte('T'));
                            }
                            else if (!NodesArray.Array[i, j].IsObstacle && NodesArray.Array[i, j] != from
                                     && NodesArray.Array[i, j] != to)
                            {
                                pathVisualizer.WriteByte(Convert.ToByte('.'));
                            }
                            else if (NodesArray.Array[i, j] == from)
                            {
                                pathVisualizer.WriteByte(Convert.ToByte('S'));
                            }
                            else
                            {
                                pathVisualizer.WriteByte(Convert.ToByte('G'));
                            }
                        }
                        pathVisualizer.WriteByte(Convert.ToByte('\n'));
                    }
                    pathVisualizer.Close();
                }
            //Write results in console
                var result = "Not Accurate";
                if (Math.Abs(length - optimalLength) < 0.0001)
                {
                    result = "Perfect";
                    ++successCounter;
                }
                Console.WriteLine("Case " + caseNumber + " Start " + splitLine[4] + " " + splitLine[5] + " Goal " + " " +
                                  splitLine[6] + " " + splitLine[7] + " Optimal Lenght = " + optimalLength 
                                  + " Length = " + length + " " + result);
            }
            Console.WriteLine(successCounter + " Absolutely accurate out of " + caseNumber);
            Console.ReadKey();
        }
    }
}
