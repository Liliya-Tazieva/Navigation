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

		public static List<Node> Inverse_Destination (List<Node> path){ 
			foreach (var node in path) {
				if (node.DestinationToStart == Destination.Up_right) {
					node.DestinationToStart = Destination.Down_left;
				}
				if (node.DestinationToStart == Destination.Down_left) {
					node.DestinationToStart = Destination.Up_right;
				}
				if (node.DestinationToStart == Destination.Down_right) {
					node.DestinationToStart = Destination.Up_left;
				}
				if (node.DestinationToStart == Destination.Up_left) {
					node.DestinationToStart = Destination.Down_right;
				}
				if (node.DestinationToStart == Destination.Up) {
					node.DestinationToStart = Destination.Down;
				}
				if (node.DestinationToStart == Destination.Left) {
					node.DestinationToStart = Destination.Right;
				}
				if (node.DestinationToStart == Destination.Down) {
					node.DestinationToStart = Destination.Up;
				}
				if (node.DestinationToStart == Destination.Right) {
					node.DestinationToStart = Destination.Left;
				}
			}
			path.Reverse();
			return path;
		}
    }
}