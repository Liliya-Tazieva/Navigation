using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using Accord.Math;
using Assets.Scripts.Core;


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

        public bool IsEmpty(int r, int c)
        {
            throw new NotImplementedException();
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
                var newSuccessors = Extensions.Neighbours(currentNode, NodesArray, null);
                foreach (var successor in newSuccessors)
                {
                    if (!OpenList.Exists(arg => arg.Currentnode.Position == successor.Currentnode.Position))
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
                        var idx = OpenList.FindIndex(arg => arg.Currentnode.Position == successor.Currentnode.Position);
                        if (OpenList[idx].Currentnode.Distance > successor.Currentnode.Distance)
                        {
                            OpenList.RemoveAt(idx);
                            successor.Currentnode.DestinationFromStart = currentNode.Currentnode.DestinationFromStart;
                            successor.Currentnode.Iteration = CurrentIteration;
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
