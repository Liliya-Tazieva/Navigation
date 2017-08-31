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
                for (var j = 0; j < widght; ++j)
                {
                    var symbol = Convert.ToChar(file.Read());
                    nodesArray[i, j] = new Node(i, j, symbol);
                }
            }
            NodesArray = new NodeArray(height, widght, nodesArray);
        }

        public static List<Node> AStar(Node from, Node to)
        {
            var finalPath = new List<Node>();
            if (from.IsObstacle || to.IsObstacle)
            {
                Console.WriteLine("Can't run A*! Enter proper from and to!");
                return finalPath;
            }
            var current = new Node(from) {Distance = Node.MetricsAStar(@from, to), Visited = NodeState.Processed};
            var observed = new List<Node> {current};

            while (current != to)
            {
                var query = NodesArray.Neighbours(current);
                query = query.Where(node => !node.IsObstacle).ToList();
                foreach (var node in query)
                {
                    if (!observed.Exists(arg => arg.Position == node.Position))
                    {
                        node.Distance = Node.MetricsAStar(node, to);
                        node.Visited = NodeState.Discovered;
                        observed.Add(node);
                    }
                }
                observed = observed.OrderBy(arg => arg.Visited).ThenBy(arg => arg.Distance).ToList();
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
                    foreach (var point in points)
                    {
                        var coordinatesX = Convert.ToInt32(point.X);
                        var coordinatesY = Convert.ToInt32(point.Y);
                        finalPath.Add(NodesArray.Array[coordinatesX, coordinatesY]);
                    }
                    i = maxIndex;
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
            while ((line = tasks.ReadLine()) != null)
            {
                string[] splitLine = line.Split('\t');
                var from = NodesArray.Array[Convert.ToInt32(splitLine[4]), Convert.ToInt32(splitLine[5])];
                var to = NodesArray.Array[Convert.ToInt32(splitLine[6]), Convert.ToInt32(splitLine[7])];
                var optimalLength = Convert.ToDouble(splitLine[8], new CultureInfo("en-US"));
                var path = AStar(from, to);
                var length = PathLength(path);
                Console.WriteLine(splitLine[4] + " " + splitLine[5] + " " + splitLine[6] + " " +
                              splitLine[7] + " Optimal Lenght = " + optimalLength + " Length = " + length + " Path: " + path.Count);
            }
            tasks.Close();

            Console.ReadKey();
        }
    }
}
