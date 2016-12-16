using System;
using System.Collections.Generic;
using System.Linq;
using Accord.MachineLearning.Structures;
using Assets.Scripts.PathFinding;
using UnityEditor;
using UnityEngine;

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

        public static float Metrics( this Informer from, Informer to ) {
            var metric = ( to.transform.position - from.transform.position ).sqrMagnitude;
            return metric;
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
            current.Distance = current.InformerNode.Metrics( to.InformerNode );
            var path = new List<Informer> { current.InformerNode };
            while ( current.InformerNode != to.InformerNode ) {
                var query = nodesTree.Nearest( current.InformerNode.transform.position.ToArray(), radius ).ToList();
                query =
                    query.Where(
                        informer => informer.InformerNode != current.InformerNode
                            && informer.InformerNode.IsObstacle != true )
                        .ToList();
                foreach ( var informer in query ) {
                    informer.Distance = informer.InformerNode.Metrics( to.InformerNode );
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

        public static List<Tree_Node> Neighbours(int x, int y, Node[,] array, Node finish, StraightLinesFromNode linesFromFinish)
        {
            var neighbours = Neighbours(x,y,array,Destinations.Default, finish, linesFromFinish);
            return neighbours;
        }

        public static List<Tree_Node> Neighbours(int x, int y, Node[,] array, Destinations destination, Node finish,
            StraightLinesFromNode linesFromFinish)
        {
            var neighbours = new List<Tree_Node>();
            var parent = array[x, y];

            //Left
            if (!array[x - 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Down ||
                    destination == Destinations.UpLeft || destination == Destinations.DownLeft)
                {
                    var delta = array[x, y].NormMatrix[1, 0] ;
                    var node = array[x - delta, y];
                    node.DestinationFromPrevious = Destinations.Left;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent,node));
                }
            }
            //Up-left
            if (!array[x - 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.UpLeft)
                {
                    var delta = array[x, y].NormMatrix[0, 0];
                    var node = array[x - delta, y + delta];
                    node.DestinationFromPrevious = Destinations.UpLeft;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            //Up
            if (!array[x, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Right ||
                    destination == Destinations.UpLeft || destination == Destinations.UpRight)
                {
                    var delta = array[x, y].NormMatrix[0, 1];
                    var node = array[x, y + delta];
                    node.DestinationFromPrevious = Destinations.Up;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            //Up-right
            if (!array[x + 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Up || destination == Destinations.UpRight)
                {
                    var delta = array[x, y].NormMatrix[0, 2];
                    var node = array[x + delta, y + delta];
                    node.DestinationFromPrevious = Destinations.UpRight;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            //Right
            if (!array[x + 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                   destination == Destinations.Up || destination == Destinations.Down ||
                   destination == Destinations.UpRight || destination == Destinations.DownRight)
                {
                    var delta = array[x, y].NormMatrix[1, 2];
                    var node = array[x + delta, y];
                    node.DestinationFromPrevious = Destinations.Right;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            //Down-right
            if (!array[x + 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Down || destination == Destinations.DownRight)
                {
                    var delta = array[x, y].NormMatrix[2, 2];
                    var node = array[x + delta, y - delta];
                    node.DestinationFromPrevious = Destinations.DownRight;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));    
                }
            }
            //Down
            if (!array[x, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                   destination == Destinations.Right || destination == Destinations.Down ||
                   destination == Destinations.DownLeft || destination == Destinations.DownRight)
                {
                    var delta = array[x, y].NormMatrix[2, 1];
                    var node = array[x, y - delta];
                    node.DestinationFromPrevious = Destinations.Down;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            //Down-left
            if (!array[x - 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Down || destination == Destinations.DownLeft)
                {
                    var delta = array[x, y].NormMatrix[2, 0];
                    var node = array[x - delta, y - delta];
                    node.DestinationFromPrevious = Destinations.DownLeft;
                    if (delta != 0) neighbours.Add(new Tree_Node(parent, node));
                }
            }
            foreach (var node in neighbours)
            {
                node.Currentnode.Distance = Metrics(node.Currentnode.InformerNode, finish.InformerNode);
                node.Currentnode.Visited = NodeState.Discovered;
                var tempNode = IsTargetJP(node.Currentnode, linesFromFinish);
                node.Currentnode.TargetJP = tempNode.TargetJP;
                node.Currentnode.DestinationToFinish = tempNode.DestinationToFinish;
            }
            neighbours = neighbours.OrderBy(arg => arg.Currentnode.Distance).ToList();
            neighbours = neighbours.OrderByDescending(arg => arg.Currentnode.TargetJP).
                ThenByDescending(arg => arg.Currentnode.JumpPoint).ToList();

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
                    Debug.Log("Point "+point.X+" "+point.Y);
                    Debug.Log("line "+line.Start.X+" "+line.Start.Y+" "+line.Finish.X+" "+line.Finish.Y);
                    isTargetJP.TargetJP = true;
                    isTargetJP.DestinationToFinish = DestinationInverse(line.Destination);
                    break;
                }
            }
            return isTargetJP;
        }

    }
}