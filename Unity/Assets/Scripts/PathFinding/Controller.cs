using System.Collections.Generic;
using System.Linq;
using Accord.MachineLearning.Structures;
using Assets.Scripts.Core;
using JetBrains.Annotations;
using UnityEngine;
using System;

namespace Assets.Scripts.PathFinding {
    public class Controller : MonoBehaviour {

        public DebugManager DebugManagerAStar;

        [UsedImplicitly]
        public Vector3 From;

        [UsedImplicitly]
        public Vector3 To;

        [UsedImplicitly]
        public float Radius;

        public KDTree<Informer> NodesTree = new KDTree<Informer>(3);
		public Node [,] NodesArray = new Node [34,35];
		public bool IsPrecomputed;
        public List<Node> JumpPoints = new List<Node>();

        public void RegisterInformer(Informer informer) {
            var position = informer.transform.position;
            NodesTree.Add(position.ToArray(), informer);
			NodesArray [(int)position.x / 3, (int)position.z / 3] = new Node (informer,NodeState.Undiscovered);
        }

        //Avoid at all costs!
        public void RemoveEmpty() {
            var newNodesTree = new KDTree<Informer>(3);
            foreach (var kdTreeNode in NodesTree) {
                if (kdTreeNode.Value != null && kdTreeNode.Value) {
                    newNodesTree.Add(kdTreeNode.Position, kdTreeNode.Value);
                }
            }
            NodesTree = newNodesTree;
        }

        public void InitializeDebugInfo() {
            if (DebugManagerAStar == null) {
                DebugManagerAStar = GetComponent<DebugManager>();
            }
        }

        [UsedImplicitly]
        public List<Informer> AStar(Informer from, Informer to, float radius) {
            DebugInformationAlgorithm debugInformation;
            var finalPath = AStar(from, to, radius, false, out debugInformation);
            return finalPath;
        }

        public List<Informer> AStar(Informer from, Informer to, float radius, bool debugFlag,
            out DebugInformationAlgorithm debugInformation)
        {
            if (from == null || to == null) {
                Debug.LogError("Can't run A*. Enter proper from and to parameters!");
                debugInformation = null;
                return null;
            }
            //Debug.Log("From: " + from.transform.position);
            //Debug.Log("To: " + to.transform.position);
            if (debugFlag) {
                debugInformation = new DebugInformationAlgorithm
                {
                    From = from,
                    To = to,
                    Observed = new List<Node>(),
                    FinalPath = new List<Informer>(),
                    LinesFromFinish = new List<Node>(),
                    CrossPoints = new List<Node>()
                };
            } else {
                debugInformation = null;
            }
            var current = new Node(from, NodeState.Processed);
            current.Distance = current.InformerNode.MetricsAStar(to);
            var observed = new List<Node> {current};
            // ReSharper disable once PossibleNullReferenceException

            while (current.InformerNode != to) {
                var query = NodesTree.Nearest(current.InformerNode.transform.position.ToArray(), radius).ToList();
                query =
                    query.Where(
                        informer => informer.InformerNode.transform.position != current.InformerNode.transform.position
                        && informer.InformerNode.IsObstacle != true)
                        .ToList();
                foreach (var informer in query) {
                    if (
                    !observed.Exists(
                        arg => arg.InformerNode.transform.position == informer.InformerNode.transform.position)) {
                            informer.Distance = informer.InformerNode.MetricsAStar(to);
                    informer.Visited = NodeState.Discovered;
                    observed.Add(informer);
                    }
                }
                observed = observed.OrderBy(arg => arg.Visited).ThenBy(arg => arg.Distance).ToList();
                if ( observed[0].Visited != NodeState.Processed ) {
                    current = observed[0];
                    observed[0].Visited = NodeState.Processed;
                    if (debugInformation != null) {
                        debugInformation.Observed.Add(observed[0]);
                    }
                } else {
                    Debug.Log("No path was found");
                    debugInformation = null;
                    return null;
                }
            }
            observed = observed.Where( informer => informer.Visited == NodeState.Processed ).ToList();
            var finalPath = new List<Informer>();
                var path = new List<Node> {current};
                while (current.InformerNode != from) {
                    var temp = current;
                    var tempFrom = temp.InformerNode.MetricsAStar(from);
                    var flag = false;
                    foreach (var informer in  observed) {
                        if (informer.InformerNode.MetricsAStar(current.InformerNode) < 18.1 &&
                            informer.Visited == NodeState.Processed) {
                                var informerFrom = informer.InformerNode.MetricsAStar(from);
                            if (tempFrom > informerFrom
                                || tempFrom <= informerFrom && flag == false) {
                                if (flag) {
                                    observed.Find(arg => arg.InformerNode.transform.position
                                                         == temp.InformerNode.transform.position).Visited =
                                        NodeState.Processed;
                                }
                                informer.Visited = NodeState.Undiscovered;
                                temp = informer;
                                tempFrom = temp.InformerNode.MetricsAStar(from);
                                flag = true;
                            }
                        }
                    }
                    if (!flag) {
                        path.RemoveAt(path.Count - 1);
                        current = path[path.Count - 1];
                    } else {
                        path.Add(temp);
                        current = temp;
                    }
                }


            Debug.Log("Path: " + path.Count);
            finalPath.Add(path[0].InformerNode);
            for (var i = 1; i < path.Count; ++i)
            {
                var maxIndex = i;
                for (var j = i; j < path.Count; ++j)
                {
                    if (StraightLine.OnOneLine(path[i], path[j], NodesArray)) maxIndex = j;
                }
                if (maxIndex != i)
                {
                    var points = StraightLine.FindMiddlePoints(path[i], path[maxIndex]);
                    foreach (var point in points)
                    {
                        finalPath.Add(NodesArray[point.X,point.Y].InformerNode);
                    }
                    i = maxIndex;
                }
            }
            finalPath.Reverse();
            Debug.Log("Final path: " +finalPath.Count);

            /*var loopflag = false;
                Node loopstart = null;
                for (var i = path.Count - 1; i >= 0; --i) {
                    var neighbours = NodesTree.Nearest(path[i].InformerNode.transform.position.ToArray(), radius).ToList();
                    var intersection = 0;
                    foreach (var informer in neighbours) {
                        if (
                            path.Exists(
                                arg => arg.InformerNode.transform.position == informer.InformerNode.transform.position))
                            ++intersection;
                    }
                    if (intersection > 3) {
                        if (!loopflag) {
                            loopflag = true;
                            int index;
                            if (i < path.Count - 1) {
                                index = i + 1;
                            } else {
                                index = i;
                            }
                            loopstart = path[index];
                            finalPath.Remove( loopstart.InformerNode );
                            //Debug.Log("Loopstart: " + loopstart.InformerNode.transform.position);
                        }
                    } else {
                        int index;
                        if (i > 0) {
                            index = i - 1;
                        } else {
                            index = i;
                        }
                        intersection = 0;
                        neighbours = NodesTree.Nearest(path[index].InformerNode.transform.position.ToArray(), radius).ToList();
                        foreach (var informer in neighbours) {
                            if (
                                path.Exists(
                                    arg => arg.InformerNode.transform.position == informer.InformerNode.transform.position))
                                ++intersection;
                        }
                        if (intersection <= 3) {
                            if (loopflag) {
                                loopflag = false;
                                var loopend = path[i];
                                //Debug.Log("Loopend: " + loopend.InformerNode.transform.position);
                                var loopescape = Extensions.LoopEscape( loopstart, loopend, NodesTree, radius);
                                if (loopescape != null) {
                                    finalPath.AddRange(loopescape);
                                }
                                loopstart = null;
                            } else {
                                finalPath.Add(path[i].InformerNode);
                            }
                        }
                    }
                }*/
            if (debugInformation != null) {
                debugInformation.FinalPath = finalPath;
            }
                /*Debug.Log("Final Path:");
                foreach (var informer in finalPath) {
                    Debug.Log(informer.transform.name + " " + informer.transform.position);
                }*/
            return finalPath;
        }
    
		public void PrecomputeMap()
		{
            JumpPoints.Clear();
		    JumpPoints = Extensions.FindPrimaryJPWithObstacles(NodesArray);

            //computing distances to jump points and obstacles
            for (var i = 0; i < 34; ++i) {
				for (var j = 0; j < 35; ++j) {
				    if (NodesArray[i, j].InformerNode.IsObstacle) continue;
				    //Checking up
				    var k = 1;
				    while (j + k < 35)
				    {
                        if (NodesArray[i, j + k].IsJumpPoint == JPType.Primary || NodesArray[i, j + k].InformerNode.IsObstacle)
				        {
                            if (NodesArray[i, j + k].IsJumpPoint == JPType.Primary)
				            {
				                NodesArray[i, j].NormMatrix[0, 1] = k;
                                if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    NodesArray[i,j].IsJumpPoint = JPType.Diagonal;
				            }
				            else
				            {
				                NodesArray[i, j].NormMatrix[0, 1] = -(k - 1);
				            }
				            break;
				        }
				        if (j + k == 34)
				        {
				            NodesArray[i, j].NormMatrix[0, 1] = -k;
				        }
				        k++;
				    }
				    //Checking down
				    k = 1;
				    while (j - k >= 0)
				    {
                        if (NodesArray[i, j - k].IsJumpPoint == JPType.Primary || NodesArray[i, j - k].InformerNode.IsObstacle)
				        {
                            if (NodesArray[i, j - k].IsJumpPoint == JPType.Primary)
				            {
				                NodesArray[i, j].NormMatrix[2, 1] = k;
                                if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                            }
				            else
				            {
				                NodesArray[i, j].NormMatrix[2, 1] = -(k - 1);
				            }
				            break;
				        }
				        if (j - k == 0)
				        {
				            NodesArray[i, j].NormMatrix[2, 1] = -k;
				        }
				        k++;
				    }
				    //Checking right
				    k = 1;
				    while (i + k < 34)
				    {
                        if (NodesArray[i + k, j].IsJumpPoint == JPType.Primary || NodesArray[i + k, j].InformerNode.IsObstacle)
				        {
                            if (NodesArray[i + k, j].IsJumpPoint == JPType.Primary)
				            {
				                NodesArray[i, j].NormMatrix[1, 2] = k;
                                if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                            }
				            else
				            {
				                NodesArray[i, j].NormMatrix[1, 2] = -(k - 1);
				            }
				            break;
				        }
				        if (i + k == 33)
				        {
				            NodesArray[i, j].NormMatrix[1, 2] = -k;
				        }
				        k++;
				    }
				    //Checking left
				    k = 1;
				    while (i - k >= 0)
				    {
                        if (NodesArray[i - k, j].IsJumpPoint == JPType.Primary || NodesArray[i - k, j].InformerNode.IsObstacle)
				        {
                            if (NodesArray[i - k, j].IsJumpPoint == JPType.Primary)
				            {
				                NodesArray[i, j].NormMatrix[1, 0] = k;
                                if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                            }
				            else
				            {
				                NodesArray[i, j].NormMatrix[1, 0] = -(k - 1);
				            }
				            break;
				        }
				        if (i - k == 0)
				        {
				            NodesArray[i, j].NormMatrix[1, 0] = -k;
				        }
				        k++;
                    }
				}
			}

                //Finding diagonal JP
		    for (var i = 0; i < 34; ++i)
		    {
		        for (var j = 0; j < 35; ++j)
		        {
                    if (NodesArray[i, j].InformerNode.IsObstacle) continue;
		            //Checking up-right
		            var k = 1;
		            if (!NodesArray[i + 1, j].InformerNode.IsObstacle && !NodesArray[i, j + 1].InformerNode.IsObstacle)
		            {
		                while (i + k < 34 && j + k < 35)
		                {
		                    if (NodesArray[i + k, j + k].IsJumpPoint == JPType.Primary ||
		                        NodesArray[i + k, j + k].InformerNode.IsObstacle
		                        || NodesArray[i + k, j + k].NormMatrix[0, 1] > 0
		                        || NodesArray[i + k, j + k].NormMatrix[1, 2] > 0
                                || NodesArray[i + k+1, j + k].InformerNode.IsObstacle
                                || NodesArray[i + k, j + k+1].InformerNode.IsObstacle)
		                    {
		                        if (NodesArray[i + k, j + k].IsJumpPoint == JPType.Primary
		                            || NodesArray[i + k, j + k].NormMatrix[0, 1] > 0
		                            || NodesArray[i + k, j + k].NormMatrix[1, 2] > 0)
		                        {
		                            NodesArray[i, j].NormMatrix[0, 2] = k;
		                            //if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
		                                //NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
		                        }
                                else if (NodesArray[i + k + 1, j + k].InformerNode.IsObstacle
                                        || NodesArray[i + k, j + k + 1].InformerNode.IsObstacle)
		                        {
                                    NodesArray[i, j].NormMatrix[0, 2] = -k;
                                }
		                        else
		                        {
		                            NodesArray[i, j].NormMatrix[0, 2] = -(k - 1);
		                        }
		                        break;
		                    }
		                    if (i + k == 33 || j + k == 34)
		                    {
		                        NodesArray[i, j].NormMatrix[0, 2] = -k;
		                    }
		                    k++;
		                }
		            }
		            //Checking down-right
		            k = 1;
                    if (!NodesArray[i + 1, j].InformerNode.IsObstacle && !NodesArray[i, j - 1].InformerNode.IsObstacle)
                    {
                        while (i + k < 34 && j - k >= 0)
                        {
                            if (NodesArray[i + k, j - k].IsJumpPoint == JPType.Primary ||
                                NodesArray[i + k, j - k].InformerNode.IsObstacle
                                || NodesArray[i + k, j - k].NormMatrix[1, 2] > 0
                                || NodesArray[i + k, j - k].NormMatrix[2, 1] > 0
                                || NodesArray[i + k, j - k-1].InformerNode.IsObstacle
                                || NodesArray[i + k+1, j - k].InformerNode.IsObstacle)
                            {
                                if(NodesArray[i + k, j - k].IsJumpPoint == JPType.Primary
                                || NodesArray[i + k, j - k].NormMatrix[1, 2] > 0
                                || NodesArray[i + k, j - k].NormMatrix[2, 1] > 0)
                                {
                                    NodesArray[i, j].NormMatrix[2, 2] = k;
                                    //if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    //NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                                }
                                else if (NodesArray[i + k, j - k - 1].InformerNode.IsObstacle
                                         || NodesArray[i + k + 1, j - k].InformerNode.IsObstacle)
                                {
                                    NodesArray[i, j].NormMatrix[2, 2] = -k;
                                }
                                else
                                {
                                    NodesArray[i, j].NormMatrix[2, 2] = -(k - 1);
                                }
                                break;
                            }
                            if (i + k == 33 || j - k == 0)
                            {
                                NodesArray[i, j].NormMatrix[2, 2] = -k;
                            }
                            k++;
                        }
                    }
		            //Checking up-left
		            k = 1;
                    if (!NodesArray[i - 1, j].InformerNode.IsObstacle && !NodesArray[i, j + 1].InformerNode.IsObstacle)
                    {
                        while (i - k >= 0 && j + k < 35)
                        {
                            if (NodesArray[i - k, j + k].IsJumpPoint == JPType.Primary ||
                                NodesArray[i - k, j + k].InformerNode.IsObstacle
                                || NodesArray[i - k, j + k].NormMatrix[0, 1] > 0
                                || NodesArray[i - k, j + k].NormMatrix[1, 0] > 0
                                || NodesArray[i - k-1, j + k].InformerNode.IsObstacle
                                || NodesArray[i - k, j + k+1].InformerNode.IsObstacle)
                            {
                                if (NodesArray[i - k, j + k].IsJumpPoint == JPType.Primary
                                    || NodesArray[i - k, j + k].NormMatrix[0, 1] > 0
                                    || NodesArray[i - k, j + k].NormMatrix[1, 0] > 0)
                                {
                                    NodesArray[i, j].NormMatrix[0, 0] = k;
                                    //if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                    //NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                                }
                                else if (NodesArray[i - k - 1, j + k].InformerNode.IsObstacle
                                        || NodesArray[i - k, j + k + 1].InformerNode.IsObstacle)
                                {
                                    NodesArray[i, j].NormMatrix[0, 0] = -k;
                                }
                                else
                                {
                                    NodesArray[i, j].NormMatrix[0, 0] = -(k - 1);
                                }
                                break;
                            }
                            if (i - k == 0 || j + k == 34)
                            {
                                NodesArray[i, j].NormMatrix[0, 0] = -k;
                            }
                            k++;
                        }
                    }
		            //Checking down-left
		            k = 1;
                    if (!NodesArray[i - 1, j].InformerNode.IsObstacle && !NodesArray[i, j - 1].InformerNode.IsObstacle)
                    {
                        while (i - k >= 0 && j - k >= 0)
                        {
                            if (NodesArray[i - k, j - k].IsJumpPoint == JPType.Primary ||
                                NodesArray[i - k, j - k].InformerNode.IsObstacle
                                || NodesArray[i - k, j - k].NormMatrix[1, 0] > 0
                                || NodesArray[i - k, j - k].NormMatrix[2, 1] > 0
                                || NodesArray[i - k-1, j - k].InformerNode.IsObstacle
                                || NodesArray[i - k, j - k-1].InformerNode.IsObstacle)
                            {
                                if (NodesArray[i - k, j - k].IsJumpPoint == JPType.Primary
                                    || NodesArray[i - k, j - k].NormMatrix[1, 0] > 0
                                    || NodesArray[i - k, j - k].NormMatrix[2, 1] > 0)
                                {
                                    NodesArray[i, j].NormMatrix[2, 0] = k;
                                    //if (NodesArray[i, j].IsJumpPoint != JPType.Primary)
                                        //NodesArray[i, j].IsJumpPoint = JPType.Diagonal;
                                }
                                else if (NodesArray[i - k - 1, j - k].InformerNode.IsObstacle
                                         || NodesArray[i - k, j - k - 1].InformerNode.IsObstacle)
                                {
                                    NodesArray[i, j].NormMatrix[2, 0] = -k;
                                }
                                else
                                {
                                    NodesArray[i, j].NormMatrix[2, 0] = -(k - 1);
                                }
                                break;
                            }
                            if (i - k == 0 || j - k == 0)
                            {
                                NodesArray[i, j].NormMatrix[2, 0] = -k;
                            }
                            k++;
                        }
                    }
                    var tileText = NodesArray[i, j].InformerNode.GetComponentInChildren<TextMesh>();
                    string text = "";
                    for (int m = 0; m < 3; ++m)
                    {
                        for (int n = 0; n < 3; ++n)
                            text = text + NodesArray[i, j].NormMatrix[m, n] + " ";
                        text = text + "\n";
                    }

                    tileText.text = text;
		        }
		    }
            //Show DiagonalJP
		    /*for (var i = 0; i < 34; ++i)
		    {
		        for (var j = 0; j < 35; ++j)
		        {
                    if(NodesArray[i, j].IsJumpPoint == JPType.Diagonal)
                    { var renderer = NodesArray[i,j].InformerNode.GetComponent<Renderer>();
                    renderer.material.SetColor("_Color", Color.magenta);}
                }
		    }*/

		    IsPrecomputed = true;
		}

        public List<Informer> JPS(Informer from, Informer to)
        {
            DebugInformationAlgorithm debugInformation;
            var finalPath = JPS(from, to, false, out debugInformation);
            return Extensions.ToInformers(finalPath);

        }

        public List<Node> JPS(Informer from, Informer to, bool debugFlag, out DebugInformationAlgorithm debugInformation)
        { 
            if (from == null || to == null)
            {
                Debug.LogError("Can't run JPS+. Enter proper from and to parameters!");
                debugInformation = null;
                return null;
            }

			if (!IsPrecomputed) {
				Debug.Log ("Precomputing...");
				PrecomputeMap ();
                Debug.Log("Done!");
			}
		    var finish = NodesArray[(int)to.transform.position.x/3, (int)to.transform.position.z/3];
            var linesFromFinish = new StraightLinesFromNode(finish);

            var start = new Tree_Node(null,NodesArray[(int)from.transform.position.x/3, (int)from.transform.position.z/3]);
		    start.Currentnode.Distance = Extensions.Metrics(start,finish);
		    var current = start;
		    var path = new List<Tree_Node>();
		    var observed = new List<Tree_Node> {current};

            if (debugFlag)
            {
                debugInformation = new DebugInformationAlgorithm
                {
                    From = from,
                    To = to,
                    Observed = new List<Node>(),
                    FinalPath = new List<Informer>(),
                    LinesFromFinish = StraightLinesFromNode.ToList(linesFromFinish,NodesArray)
                };
            }
            else
            {
                debugInformation = null;
            }

		    while (current.Currentnode != finish)
		    {
                
                if (!observed.Exists(arg => arg.Currentnode.Visited!=NodeState.Processed))
		        {
                    Debug.Log("No path was found");
                    if (debugFlag)
                    {
                        debugInformation.Observed = Extensions.ToNodes(
                            observed.Where(arg => arg.Currentnode.Visited == NodeState.Processed).
                            OrderBy(arg => arg.Level).ToList());
                    }
                    return null;
		        }
                observed[0].Currentnode.Visited = NodeState.Processed;


                //Go to finish if in Target JP
                current.Currentnode = Extensions.IsTargetJP(current.Currentnode, linesFromFinish);
                if (current.Currentnode.TargetJP && Extensions.Reachable(current.Currentnode, finish, NodesArray))
		        {
		            finish.DestinationFromPrevious = Extensions.DestinationInverse(current.Currentnode.DestinationToFinish);
		            path.Add(current);
		            current = new Tree_Node(current, finish);
		            path.Add(current);
		            break;
		        }

                //Find next nodes

                //Neighbours
                var neighbours = Extensions.Neighbours(current, NodesArray, finish, linesFromFinish);

                //Target JP
                var lines = new StraightLinesFromNode(current.Currentnode,Extensions.GetDestinationsFromNeighbours(neighbours));

                var minMetrics = current.Currentnode.Distance;
                var tempList = new List<Node>();
                if (lines.Lines != null)
                {
                    foreach (var lineFromFinish in linesFromFinish.Lines)
                    {
                        foreach (var line in lines.Lines)
                        {
                            var coordinates = StraightLine.Crossing(line, lineFromFinish);
                            if(coordinates!=null &&
                                Extensions.Reachable(current.Currentnode, NodesArray[coordinates.X, coordinates.Y], NodesArray))
                            if (coordinates != null && Extensions.Reachable(NodesArray[coordinates.X, coordinates.Y], finish, NodesArray)
                                && Extensions.Reachable(current.Currentnode,NodesArray[coordinates.X,coordinates.Y],NodesArray))
                            {
                                var tempNode = new Node(NodesArray[coordinates.X, coordinates.Y]);
                                tempNode.Distance = Extensions.Metrics(new Tree_Node(current, tempNode), finish);
                                tempNode.TargetJP = true;
                                tempNode.DestinationToFinish = Extensions.DestinationInverse(lineFromFinish.Destination);
                                tempNode.Visited = NodeState.Discovered;
                                if (tempNode.Distance < minMetrics)
                                {
                                    minMetrics = tempNode.Distance;
                                    tempList.Clear();
                                    tempList.Add(tempNode);
                                }
                                else if (Math.Abs(tempNode.Distance - minMetrics) < 0.00000000001)
                                {
                                    tempNode.Distance = Extensions.Metrics(new Tree_Node(current, tempNode), finish);
                                    tempList.Add(tempNode);
                                }
                            }
                        }
                    }
                }
                Tree_Node tempTargetJP = null;
                if (tempList.Count != 0)
                {
                    tempTargetJP = new Tree_Node(current, tempList[0]);
                    if (tempList.Count > 1)
                    {
                        var min = tempTargetJP.DistanceFromParent;
                        foreach (var node in tempList)
                        {
                            var tempMetrics = current.Currentnode.InformerNode.MetricsAStar(node.InformerNode);
                            if (tempMetrics < min)
                            {
                                tempTargetJP = new Tree_Node(current, node);
                                min = tempMetrics;
                            }
                        }
                    }
                    if (debugFlag)
                    {
                        debugInformation.CrossPoints.Add(tempTargetJP.Currentnode);
                    }
                    if (!observed.Exists(arg => arg.Currentnode.Position == tempTargetJP.Currentnode.Position))
                        observed.Add(tempTargetJP);
                    else
                    {
                        var index =
                            observed.FindIndex(arg => arg.Currentnode.Position == tempTargetJP.Currentnode.Position);
                        if (observed[index].Currentnode.Visited == NodeState.Discovered)
                            observed[index].Currentnode.Distance = tempTargetJP.Currentnode.Distance;
                    }
                }
                if (neighbours.Count != 0)
                {
                    foreach(var neighbour in neighbours)
                    {
                        if (!observed.Exists(arg => arg.Currentnode.Position == neighbour.Currentnode.Position))
                        {
                            if (Extensions.SelectJPFromNeighbours(current,neighbour)) observed.Add(neighbour);
                        }
                        else
                        {
                            var index =
                                observed.FindIndex(arg => arg.Currentnode.Position == neighbour.Currentnode.Position);
                            if (observed[index].Currentnode.Visited == NodeState.Discovered)
                                observed[index].Currentnode.Distance = neighbour.Currentnode.Distance;
                        }
                    }
                }
                observed = observed.OrderBy(arg => arg.Currentnode.Visited).
                    ThenBy(arg => arg.Currentnode.Distance).ToList();
                path.Add(current);

		        if (!observed[0].Currentnode.TargetJP && tempTargetJP != null)
		        {
		            current = observed[0].DistanceFromParent < tempTargetJP.DistanceFromParent ? observed[0] : tempTargetJP;
		        }
		        else current = observed[0];


		    }
            Debug.Log("Path: "+path.Count);
            if(path.Count>1)
            {
                var finalPath = new List<Node>();
                while (current!=start)
                {
                    var middlePoints = StraightLine.FindMiddlePoints(current.Parent, current.Currentnode);
                    if(current.Parent!=start.Currentnode) middlePoints.RemoveAt(0);
                    finalPath.InsertRange(0,Extensions.ToNodes(middlePoints, NodesArray));
                    current = path.Find(arg => arg.Currentnode.Position == current.Parent.Position &&
                    arg.Level == current.Level-1);
                }

                if (debugFlag)
                {
                    debugInformation.Observed = Extensions.ToNodes(
                        observed.Where(arg => arg.Currentnode.Visited==NodeState.Processed).
                        OrderBy(arg => arg.Level).ToList());
                    debugInformation.FinalPath = Extensions.ToInformers(finalPath);
                }
                Debug.Log("Final path: " + finalPath.Count);

                return finalPath;
		    }
            else return null;
		}
    }
}