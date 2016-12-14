using System;
using System.Collections.Generic;
using System.Linq;
using Accord.MachineLearning.Structures;
using Assets.Scripts.PathFinding;
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

        public static List<Node> Neighbours(int x, int y, Node[,] array)
        {
            var neighbours = Neighbours(x,y,array,Destinations.Default);
            return neighbours;
        }

        public static List<Node> Neighbours(int x, int y, Node[,] array, Destinations destination)
        {
            var neighbours = new List<Node>();

            //Left
            if (!array[x - 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Down ||
                    destination == Destinations.UpLeft || destination == Destinations.DownLeft)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[1, 0]) ;
                    neighbours.Add(array[x - delta, y]);
                }
            }
            //Up-left
            if (!array[x - 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.UpLeft)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[0, 0]);
                    neighbours.Add(array[x - delta, y + delta]);
                }
            }
            //Up
            if (!array[x, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Up || destination == Destinations.Right ||
                    destination == Destinations.UpLeft || destination == Destinations.UpRight)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[0, 1]);
                    neighbours.Add(array[x, y + delta]);
                }
            }
            //Up-right
            if (!array[x + 1, y + 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Up || destination == Destinations.UpRight)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[0, 2]);
                    neighbours.Add(array[x + delta, y + delta]);
                }
            }
            //Right
            if (!array[x + 1, y].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                   destination == Destinations.Up || destination == Destinations.Down ||
                   destination == Destinations.UpRight || destination == Destinations.DownRight)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[1, 2]);
                    neighbours.Add(array[x + delta, y]);
                }
            }
            //Down-right
            if (!array[x + 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Right ||
                    destination == Destinations.Down || destination == Destinations.DownRight)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[2, 2]);
                    neighbours.Add(array[x + delta, y - delta]);    
                }
            }
            //Down
            if (!array[x, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                   destination == Destinations.Right || destination == Destinations.Down ||
                   destination == Destinations.DownLeft || destination == Destinations.DownRight)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[2, 1]);
                    neighbours.Add(array[x, y - delta]);
                }
            }
            //Down-left
            if (!array[x - 1, y - 1].InformerNode.IsObstacle)
            {
                if (destination == Destinations.Default || destination == Destinations.Left ||
                    destination == Destinations.Down || destination == Destinations.DownLeft)
                {
                    var delta = Math.Abs(array[x, y].NormMatrix[2, 0]);
                    neighbours.Add(array[x - delta, y - delta]);
                }
            }
            foreach (var node in neighbours)
            {
                node.Visited = NodeState.Discovered;
            }

            return neighbours;
        }

        public static List<Node> DestinationsList(Node start, Node finish, Node [,] array)
        {
            var nodes = new List<Node>();
            if (start.X() < finish.X() && start.Y() < finish.Y())
            {
                
            }
            else if (start.X() < finish.X() && start.Y() > finish.Y())
            {

            }
            else if (start.X() > finish.X() && start.Y() < finish.Y())
            {

            }
            else if (start.X() > finish.X() && start.Y() > finish.Y())
            {

            }

            return nodes;
        }
    }
}