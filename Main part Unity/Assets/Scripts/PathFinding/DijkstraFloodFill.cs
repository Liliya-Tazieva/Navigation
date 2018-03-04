using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using Accord.Math;
using Assets.Scripts.Core;
using UnityEngine;


namespace Assets.Scripts.PathFinding
{
    class DijkstraFloodFill
    {
        public static readonly float One = 1f;
        public static readonly float Square = (float)Math.Sqrt(2);

        public int CurrentIteration;

        public List<Tree_Node> OpenList;

        public Tree_Node[,] NodesArray;

        public DijkstraFloodFill(Node[,] nodes)
        {
            CurrentIteration = 0;
            var c = nodes.Columns();
            var r = nodes.Rows();
            NodesArray = new Tree_Node[r, c];
            for (var i = 0; i < r; ++i)
                for (var j = 0; j < c; ++j)
                    NodesArray[i, j] = new Tree_Node(null, nodes[i, j]);
            OpenList = new List<Tree_Node>();
        }

        public int GetCurrentIteration()
        {
            return CurrentIteration;
        }

        private Tree_Node AddRight(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y()].Currentnode.InformerNode.IsObstacle 
                ? NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y()] : null;
        }

        private Tree_Node AddLeft(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y()].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y()] : null;
        }

        private Tree_Node AddUp(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X(), current.Currentnode.Y() + 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X(), current.Currentnode.Y() + 1] : null;
        }

        private Tree_Node AddDown(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X(), current.Currentnode.Y() - 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X(), current.Currentnode.Y() - 1] : null;
        }

        private Tree_Node AddUpRight(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y() + 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y() + 1]
                : null;
        }

        private Tree_Node AddUpLeft(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y() + 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y() + 1]
                : null;
        }

        private Tree_Node AddDownRight(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y() - 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X() + 1, current.Currentnode.Y() - 1]
                : null;
        }

        private Tree_Node AddDownLeft(Tree_Node current)
        {
            return !NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y() - 1].Currentnode.InformerNode.IsObstacle
                ? NodesArray[current.Currentnode.X() - 1, current.Currentnode.Y() - 1]
                : null;
        }

        public List<Tree_Node> AddAll(Tree_Node current)
        {
            var nodes = new List<Tree_Node>();
            nodes.Add(AddDown(current));
            nodes.Add(AddDownLeft(current));
            nodes.Add(AddDownRight(current));
            nodes.Add(AddLeft(current));
            nodes.Add(AddUp(current));
            nodes.Add(AddRight(current));
            nodes.Add(AddUpLeft(current));
            nodes.Add(AddUpRight(current));
            return nodes;
        }

        public List<Tree_Node> Neighbours(Tree_Node current)
        {
            var neighbours = new List<Tree_Node>();
            if (current.Currentnode.DestinationFromStart == Destinations.Default)
            {
                neighbours.AddRange(AddAll(current));
            }
            else if (current.Currentnode.DestinationFromStart == Destinations.Right)
            {
                var x = current.Currentnode.X();
                var y = current.Currentnode.Y();
                var node = AddRight(current);
                neighbours.Add(node);
                var isForced = false;

                if (NodesArray[x, y + 1].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddUp(node));
                    neighbours.Add(AddUpRight(node));
                    isForced = true;
                }
                if (NodesArray[x, y - 1].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddDown(node));
                    neighbours.Add(AddDownRight(node));
                    isForced = true;
                }
                if(isForced) neighbours.Add(AddRight(node));
            }
            else if (current.Currentnode.DestinationFromStart == Destinations.Left)
            {
                var x = current.Currentnode.X();
                var y = current.Currentnode.Y();
                var node = AddLeft(current);
                neighbours.Add(node);
                var isForced = false;

                if (NodesArray[x, y + 1].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddUp(node));
                    neighbours.Add(AddUpLeft(node));
                    isForced = true;
                }
                if (NodesArray[x, y - 1].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddDown(node));
                    neighbours.Add(AddDownLeft(node));
                    isForced = true;
                }
                if (isForced) neighbours.Add(AddLeft(node));
            }
            else if (current.Currentnode.DestinationFromStart == Destinations.Up)
            {
                var x = current.Currentnode.X();
                var y = current.Currentnode.Y();
                var node = AddUp(current);
                neighbours.Add(node);
                var isForced = false;

                if (NodesArray[x + 1, y].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddRight(node));
                    neighbours.Add(AddUpRight(node));
                    isForced = true;
                }
                if (NodesArray[x - 1, y].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddLeft(node));
                    neighbours.Add(AddUpLeft(node));
                    isForced = true;
                }
                if (isForced) neighbours.Add(AddUp(node));
            }
            else if (current.Currentnode.DestinationFromStart == Destinations.Down)
            {
                var x = current.Currentnode.X();
                var y = current.Currentnode.Y();
                var node = AddDown(current);
                neighbours.Add(node);
                var isForced = false;

                if (NodesArray[x + 1, y].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddRight(node));
                    neighbours.Add(AddDownRight(node));
                    isForced = true;
                }
                if (NodesArray[x - 1, y].Currentnode.InformerNode.IsObstacle)
                {
                    neighbours.Add(AddLeft(node));
                    neighbours.Add(AddDownLeft(node));
                    isForced = true;
                }
                if (isForced) neighbours.Add(AddDown(node));
            }
            else if (current.Currentnode.DestinationFromStart == Destinations.UpRight)
            {

            }
            else if (current.Currentnode.DestinationFromStart == Destinations.UpLeft)
            {

            }
            else if (current.Currentnode.DestinationFromStart == Destinations.DownRight)
            {

            }
            else if (current.Currentnode.DestinationFromStart == Destinations.DownLeft)
            {

            }
            return neighbours;
        }

        public void Flood(int startRow, int startCol)
        {
            CurrentIteration++;

            NodesArray[startRow, startCol].Currentnode.Iteration = CurrentIteration;
            NodesArray[startRow, startCol].Currentnode.Visited = NodeState.Discovered;
            OpenList.Add(NodesArray[startRow, startCol]);
            
            while (OpenList.Count != 0)
            {

                var currentNode = OpenList[0];

                // Explore nodes based on the parent and surrounding walls.
                // This must be in the search style of JPS+ in order to produce
                // the correct data for JPS+. If goal bounding is used for regular
                // A*, then this search would need to mimic regular A*.
                var newSuccessors = Neighbours(currentNode);
                foreach (var successor in newSuccessors)
                {
                    if (!OpenList.Exists(arg => arg.Currentnode.Position == successor.Currentnode.Position)
                        &&successor.Currentnode.Iteration != CurrentIteration)
                    {
                        // Place node on the Open list (we've never seen it before)
                        successor.Currentnode.DestinationFromStart =
                            currentNode.Currentnode.DestinationFromStart != Destinations.Default
                                ? currentNode.Currentnode.DestinationFromStart
                                : successor.Currentnode.DestinationFromPrevious;
                        successor.Currentnode.Iteration = CurrentIteration;
                        OpenList.Add(successor);
                    }
                    else
                    {
                        // We found a cheaper way to this node - update it
                        var row = (int)successor.Currentnode.Position.x / 3;
                        var col = (int)successor.Currentnode.Position.z / 3;
                        if (NodesArray[row,col].Currentnode.Distance > successor.Currentnode.Distance)
                        {
                            OpenList.Remove(successor);
                            successor.Currentnode.DestinationFromStart = currentNode.Currentnode.DestinationFromStart;
                            successor.Currentnode.Iteration = CurrentIteration;
                            NodesArray[row, col] = successor;
                            OpenList.Add(successor);
                        }
                    }
                }

                var rowIdx = (int)currentNode.Currentnode.Position.x / 3;
                var colIdx = (int)currentNode.Currentnode.Position.z / 3;
                OpenList.RemoveAt(0);
                currentNode.Currentnode.Visited = NodeState.Processed;
                NodesArray[rowIdx, colIdx] = currentNode;
            }
        }
    }
}
