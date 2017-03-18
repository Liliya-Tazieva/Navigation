using System.Collections.Generic;
using Assets.Scripts.Core;
using UnityEngine;

namespace Assets.Scripts.PathFinding
{
    public class Tree_Node
    {
        public Node Currentnode;
        public Node Parent;
        public float DistanceFromParent;

        public Tree_Node(Node parent, Node current)
        {
            Currentnode = current;
            Parent = parent;
            DistanceFromParent = Parent != null ? Currentnode.InformerNode.MetricsAStar(Parent.InformerNode) : 0;
            FindDestination();
        }

        public static List<Tree_Node> NodesToList(List<Node> nodes, Node parent)
        {
            var list = new List<Tree_Node>();
            foreach (var node in nodes)
            {
                var treeNode = new Tree_Node(parent,node);
                list.Add(treeNode);
            }
            return list;
        }

        public void FindDestination()
        {
            if (Parent == null) Currentnode.DestinationFromPrevious = Destinations.Default;
            else
            {
                if (Currentnode.X() > Parent.X())
                {
                    if (Currentnode.Y() > Parent.Y())
                        Currentnode.DestinationFromPrevious = Destinations.UpRight;
                    else if (Currentnode.Y() < Parent.Y()) Currentnode.DestinationFromPrevious = Destinations.DownRight;
                    else Currentnode.DestinationFromPrevious = Destinations.Right;
                }
                else if (Currentnode.X() < Parent.X())
                {
                    if (Currentnode.Y() > Parent.Y())
                        Currentnode.DestinationFromPrevious = Destinations.UpLeft;
                    else if (Currentnode.Y() < Parent.Y()) Currentnode.DestinationFromPrevious = Destinations.DownLeft;
                    else Currentnode.DestinationFromPrevious = Destinations.Left;
                }
                else
                {
                    Currentnode.DestinationFromPrevious = Currentnode.Y() > Parent.Y()
                        ? Destinations.Up
                        : Destinations.Down;
                }
            }
        }

        public static List<Node> ToNodeList(List<Tree_Node> treeNodeList)
        {
            var list = new List<Node>();
            foreach (var treeNode in treeNodeList)
            {
                list.Add(treeNode.Currentnode);
            }
            return list;
        }
    }
}