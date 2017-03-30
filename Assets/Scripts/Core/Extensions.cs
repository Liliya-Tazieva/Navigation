using System;
using System.Collections.Generic;
using System.Linq;
using Accord.MachineLearning.Structures;
using Accord.Math;
using Assets.Scripts.PathFinding;
using UnityEngine;
using UnityEngine.Assertions.Must;

namespace Assets.Scripts.Core {

    public static class Extensions {
        public static void ForEach<T>( this IEnumerable<T> from, Action<T> callback ) {
            foreach ( var element in from ) {
                callback( element );
            }
        }

        public static double[] ToArray( this Vector3 vec ) {
            return new double[] { vec.x, vec.y, vec.z };
        }

		public static Vector3 Position( this Node node ) {
			return node.InformerNode.transform.position;
		}

        public static float MetricsAStar( this Informer from, Informer to ) {
            var metric = ( to.transform.position - from.transform.position ).sqrMagnitude;
            return metric;
        }

        public static float Metrics(Tree_Node from, Node to)
        {
            if (from.Parent != null)
            {
                return ((to.Position - from.Currentnode.Position).sqrMagnitude +
                        (from.Currentnode.Position - from.Parent.Position).sqrMagnitude);
            }
            else return MetricsAStar(from.Currentnode.InformerNode, to.InformerNode);
        }

        public static List<Node> ToList( this KDTreeNodeList<Informer> tree ) {
            return tree
                .Select( node => new Node( node.Node.Value, NodeState.Undiscovered ) )
                .ToList();
        }

        public static List<Node> ToNodes(List<Point> points, Node [,] nodesArray)
        {
            var list = new List<Node>();
            for (var i=0;i<points.Count;++i)
            {
                list.Add(nodesArray[points[i].X,points[i].Y]);
            }
            return list;
        }

        public static List<Node> ToNodes(List<Tree_Node> nodes)
        {
            var list = new List<Node>();
            for (var i = 0; i < nodes.Count; ++i)
            {
                list.Add(nodes[i].Currentnode);
            }
            return list;
        }

        public static List<Informer> ToInformers(List<Node> nodes)
        {
            var list = new List<Informer>();
            for (var i = 0; i < nodes.Count; ++i)
            {
                list.Add(nodes[i].InformerNode);
            }
            return list;
        }
        
        public static List<Informer> LoopEscape( Node from, Node to, KDTree<Informer> nodesTree, float radius ) {
            var current = from;
            current.Distance = current.InformerNode.MetricsAStar( to.InformerNode );
            var path = new List<Informer> { current.InformerNode };
            while ( current.InformerNode != to.InformerNode ) {
                var query = nodesTree.Nearest( current.InformerNode.transform.position.ToArray(), radius ).ToList();
                query =
                    query.Where(
                        informer => informer.InformerNode != current.InformerNode
                            && informer.InformerNode.IsObstacle != true )
                        .ToList();
                foreach ( var informer in query ) {
                    informer.Distance = informer.InformerNode.MetricsAStar( to.InformerNode );
                }
                query = query.Where( informer => informer.Distance < current.Distance )
                .ToList().OrderBy( informer => informer.Distance ).ToList();
                current = query[0];
                path.Add( current.InformerNode );
            }
            return path;
        }

        public static Destinations DestinationInverse(Destinations destination)
        {
            if (destination == Destinations.Default) return Destinations.Default;
            else if (destination == Destinations.Right) return Destinations.Left;
            else if (destination == Destinations.Left) return Destinations.Right;
            else if (destination == Destinations.Up) return Destinations.Down;
            else if (destination == Destinations.Down) return Destinations.Up;
            else if (destination == Destinations.UpRight) return Destinations.DownLeft;
            else if (destination == Destinations.DownLeft) return Destinations.UpRight;
            else if (destination == Destinations.UpLeft) return Destinations.DownRight;
            else return Destinations.UpLeft;
        }

        public static List<Tree_Node> Neighbours(Node node, Node[,] array, Node finish, 
            StraightLinesFromNode linesFromFinish)
        {
            var neighbours = NeighboursSelective(node.X(), node.Y(), array, node.DestinationFromPrevious, finish, linesFromFinish);
            return neighbours;
        }
        public static List<Tree_Node> NeighboursSelective(int x, int y, Node[,] array, Destinations destination, Node finish,
            StraightLinesFromNode linesFromFinish)
        {
            var neighbours = new List<Tree_Node>();
            var parent = new Node(array[x, y]);


            //Left
            if (!array[x - 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Down ||
                    destination == Destinations.UpLeft || destination == Destinations.DownLeft)
                {
                    var delta = parent.NormMatrix[1, 0] ;
                    delta = Math.Abs(delta);
                    var node = new Node(array[x - delta, y]);
                    node.DestinationFromPrevious = Destinations.Left;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent,node));
                    }
                }
            }
            //Up-left
            if (!array[x - 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.UpLeft)
                {
                    var delta = parent.NormMatrix[0, 0];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x - delta, y + delta]);
                    node.DestinationFromPrevious = Destinations.UpLeft;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Up
            if (!array[x, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Right ||
                    destination == Destinations.UpLeft || destination == Destinations.UpRight)
                {
                    var delta = parent.NormMatrix[0, 1];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x, y + delta]);
                    node.DestinationFromPrevious = Destinations.Up;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Up-right
            if (!array[x + 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Up || destination == Destinations.UpRight)
                {
                    var delta = parent.NormMatrix[0, 2];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x + delta, y + delta]);
                    node.DestinationFromPrevious = Destinations.UpRight;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Right
            if (!array[x + 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                   destination == Destinations.Up || destination == Destinations.Down ||
                   destination == Destinations.UpRight || destination == Destinations.DownRight)
                {
                    var delta = parent.NormMatrix[1, 2];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x + delta, y]);
                    node.DestinationFromPrevious = Destinations.Right;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Down-right
            if (!array[x + 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Down || destination == Destinations.DownRight)
                {
                    var delta = parent.NormMatrix[2, 2];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x + delta, y - delta]);
                    node.DestinationFromPrevious = Destinations.DownRight;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Down
            if (!array[x, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                   destination == Destinations.Right || destination == Destinations.Down ||
                   destination == Destinations.DownLeft || destination == Destinations.DownRight)
                {
                    var delta = parent.NormMatrix[2, 1];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x, y - delta]);
                    node.DestinationFromPrevious = Destinations.Down;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            //Down-left
            if (!array[x - 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Down || destination == Destinations.DownLeft)
                {
                    var delta = parent.NormMatrix[2, 0];
                    delta = Math.Abs(delta);
                    var node = new Node(array[x - delta, y - delta]);
                    node.DestinationFromPrevious = Destinations.DownLeft;
                    if (delta != 0)
                    {
                        node.Visited = NodeState.Discovered;
                        neighbours.Add(new Tree_Node(parent, node));
                    }
                }
            }
            foreach (var node in neighbours)
            {
                node.Currentnode.Distance = Metrics(node, finish);
                node.Currentnode.Visited = NodeState.Discovered;
            }

            return neighbours;
        }

        public static bool GoToFinish(Node node, Node finish)
        {
            if (node.DestinationToFinish == Destinations.Right)
            {
                var delta = node.NormMatrix[1, 2];
                if(finish.X() <= node.X()+delta && finish.Y() == node.Y()) return true;
            }
            else if (node.DestinationToFinish == Destinations.Left)
            {
                var delta = node.NormMatrix[1, 0];
                if (finish.X() >= node.X() - delta && finish.Y() == node.Y()) return true;
            }
            else if (node.DestinationToFinish == Destinations.Up)
            {
                var delta = node.NormMatrix[0, 1];
                if (finish.X() == node.X() && finish.Y() <= node.Y()+delta) return true;
            }
            else if (node.DestinationToFinish == Destinations.Down)
            {
                var delta = node.NormMatrix[2, 1];
                if (finish.X() == node.X() && finish.Y() >= node.Y()-delta) return true;
            }
            else if (node.DestinationToFinish == Destinations.UpRight)
            {
                var delta = node.NormMatrix[0, 2];
                if (finish.X() <= node.X() + delta && finish.Y() <= node.Y()+delta) return true;
            }
            else if (node.DestinationToFinish == Destinations.DownLeft)
            {
                var delta = node.NormMatrix[2, 0];
                if (finish.X() >= node.X() - delta && finish.Y() >= node.Y() - delta) return true;
            }
            else if (node.DestinationToFinish == Destinations.UpLeft)
            {
                var delta = node.NormMatrix[0, 0];
                if (finish.X() >= node.X() - delta && finish.Y() <= node.Y() + delta) return true;
            }
            else if (node.DestinationToFinish == Destinations.DownRight)
            {
                var delta = node.NormMatrix[2, 2];
                if (finish.X() <= node.X() + delta && finish.Y() >= node.Y() - delta) return true;
            }
            return false;
        }

        public static Node IsTargetJP(Node node, StraightLinesFromNode lines)
        {
            var isTargetJP = node;
            foreach (var line in lines.Lines)
            {
                var point = new Point(node.X(), node.Y());
                if (point.Belongs(line))
                {
                    /*Debug.Log("Point "+point.X+" "+point.Y);
                    Debug.Log("line "+line.Start.X+" "+line.Start.Y+" "+line.Finish.X+" "+line.Finish.Y);*/
                    isTargetJP.TargetJP = true;
                    isTargetJP.DestinationToFinish = DestinationInverse(line.Destination);
                    break;
                }
            }
            return isTargetJP;
        }

        public static List<Destinations> DestinationsFromCurrent(Node node)
        {
            var destination = node.DestinationFromPrevious;
            var list = new List<Destinations>();
            if (destination == Destinations.Default || destination == Destinations.Left ||
                destination == Destinations.Up || destination == Destinations.Down ||
                destination == Destinations.UpLeft || destination == Destinations.DownLeft)
                list.Add(Destinations.Left);
            if (destination == Destinations.Default || destination == Destinations.Left ||
                destination == Destinations.Up || destination == Destinations.UpLeft)
                list.Add(Destinations.UpLeft);
            if (destination == Destinations.Default || destination == Destinations.Left ||
                destination == Destinations.Up || destination == Destinations.Right ||
                destination == Destinations.UpLeft || destination == Destinations.UpRight)
                list.Add(Destinations.Up);
            if (destination == Destinations.Default || destination == Destinations.Right ||
                destination == Destinations.Up || destination == Destinations.UpRight)
                list.Add(Destinations.UpRight);
            if (destination == Destinations.Default || destination == Destinations.Right ||
                destination == Destinations.Up || destination == Destinations.Down ||
                destination == Destinations.UpRight || destination == Destinations.DownRight)
                list.Add(Destinations.Right);
            if (destination == Destinations.Default || destination == Destinations.Right ||
                destination == Destinations.Down || destination == Destinations.DownRight)
                list.Add(Destinations.DownRight);
            if (destination == Destinations.Default || destination == Destinations.Left ||
                destination == Destinations.Right || destination == Destinations.Down ||
                destination == Destinations.DownLeft || destination == Destinations.DownRight)
                list.Add(Destinations.Down);
            if (destination == Destinations.Default || destination == Destinations.Left ||
                destination == Destinations.Down || destination == Destinations.DownLeft)
                list.Add(Destinations.DownLeft);


                return list;
        }

        public static bool Reachable(Node from, Node to, Node[,] nodesArray)
        {
            if (from.InformerNode.IsObstacle) return false;
            var pointsBetween = StraightLine.FindMiddlePoints(from, to);
            var destination = StraightLine.FindDestination(from, to);
            if (destination == Destinations.UpRight || destination == Destinations.UpLeft ||
                destination == Destinations.DownLeft || destination == Destinations.DownRight)
            {
                if (destination == Destinations.UpRight || destination == Destinations.UpLeft)
                {
                    foreach (var point in pointsBetween)
                    {
                        if (nodesArray[point.X, point.Y + 1].InformerNode.IsObstacle) return false;
                    }
                }
                else
                {
                    foreach (var point in pointsBetween)
                    {
                        if (nodesArray[point.X, point.Y - 1].InformerNode.IsObstacle) return false;
                    }
                }
            }
            foreach (var point in pointsBetween)
            {
                if (nodesArray[point.X, point.Y].InformerNode.IsObstacle) return false;
            }
            return true;
        }

        public static List<Destinations> GetDestinationsFromNeighbours(List<Tree_Node> neighbours)
        {
            var destinations = new List<Destinations>();
            foreach (var neighbour in neighbours)
            {
                destinations.Add(neighbour.Currentnode.DestinationFromPrevious);
            }
            return destinations;
        }

        public static void NeighboursAndTJP(Informer start, Informer finish, 
            Node[,] nodesArray, out DebugInformationAlgorithm debugInfo)
        {
            var startNode = new Node(start,NodeState.Processed);
            startNode.Distance = MetricsAStar(start, finish);
            var finishNode = new Node(finish, NodeState.Processed);
            var sraightLinesFromStart = new StraightLinesFromNode(startNode);
            var sraightLinesFromFinish = new StraightLinesFromNode(finishNode);
            var neighbours = Neighbours(startNode, nodesArray, finishNode, sraightLinesFromFinish);

            var minMetrics = startNode.Distance;
            var tempList = new List<Node>();
            if (sraightLinesFromStart.Lines != null)
            {
                foreach (var lineFromFinish in sraightLinesFromFinish.Lines)
                {
                    foreach (var line in sraightLinesFromStart.Lines)
                    {
                        var coordinates = StraightLine.Crossing(line, lineFromFinish);
                        if (coordinates != null && Reachable(nodesArray[coordinates.X, coordinates.Y], finishNode, nodesArray)
                            && Reachable(startNode, nodesArray[coordinates.X, coordinates.Y], nodesArray))
                        {
                            var tempNode = new Node(nodesArray[coordinates.X, coordinates.Y]);
                            tempNode.Distance = Metrics(new Tree_Node(startNode, tempNode), finishNode);
                            tempNode.TargetJP = true;
                            tempNode.DestinationToFinish = DestinationInverse(lineFromFinish.Destination);
                            tempNode.Visited = NodeState.Discovered;
                            if (tempNode.Distance < minMetrics)
                            {
                                minMetrics = tempNode.Distance;
                                tempList.Clear();
                                tempList.Add(tempNode);
                            }
                            else if (Math.Abs(tempNode.Distance - minMetrics) < 0.00000000001)
                            {
                                tempList.Add(tempNode);
                            }
                        }
                    }
                }
            }
            var straightLines = StraightLinesFromNode.JoinLines(sraightLinesFromStart, sraightLinesFromFinish,
                nodesArray);
            debugInfo = new DebugInformationAlgorithm
            {
                From = startNode.InformerNode,
                To = finishNode.InformerNode,
                Observed = ToNodes(neighbours),
                LinesFromFinish = straightLines,
                CrossPoints = tempList,
                Destroy = false
            };
        }
    }
}