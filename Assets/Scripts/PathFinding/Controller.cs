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
                    FinalPath = new List<Informer>()
                };
            } else {
                debugInformation = null;
            }
            var current = new Node(from, NodeState.Processed);
            current.Distance = current.InformerNode.Metrics(to);
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
                    informer.Distance = informer.InformerNode.Metrics(to);
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
                    var tempFrom = temp.InformerNode.Metrics(from);
                    var flag = false;
                    foreach (var informer in  observed) {
                        if (informer.InformerNode.Metrics(current.InformerNode) < 18.1 &&
                            informer.Visited == NodeState.Processed) {
                            var informerFrom = informer.InformerNode.Metrics(from);
                            if (tempFrom > informerFrom
                                || tempFrom <= informerFrom && flag == false) {
                                if (flag) {
                                    observed.Find(arg => arg.InformerNode.transform.position
                                                         == temp.InformerNode.transform.position).Visited =
                                        NodeState.Processed;
                                }
                                informer.Visited = NodeState.Undiscovered;
                                temp = informer;
                                tempFrom = temp.InformerNode.Metrics(from);
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
                var loopflag = false;
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
                }
            if (debugInformation != null) {
                debugInformation.FinalPath = finalPath;
            }
                /*Debug.Log("Final Path:");
                foreach (var informer in finalPath) {
                    Debug.Log(informer.transform.name + " " + informer.transform.position);
                }*/
            return finalPath;
        }
    
		public void PrecomputeMap(){
			//finding jump points
			for (var i = 0; i < 34; ++i) {
				for (var j = 0; j < 35; ++j) {
					if (!NodesArray [i, j].InformerNode.IsObstacle) {
						//right obstacle
						if (i < 33) {
							if (NodesArray [i + 1, j].InformerNode.IsObstacle) {
								if (j > 0) {
									if(!NodesArray [i + 1, j-1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
								if (j < 34) {
									if(!NodesArray [i + 1, j+1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
							}
						}
						//left obstacle
						if (i > 0) {
							if (NodesArray [i - 1, j].InformerNode.IsObstacle) {
								if (j > 0) {
									if(!NodesArray [i - 1, j-1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
								if (j < 34) {
									if(!NodesArray [i - 1, j+1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
							}
						}
						//up obstacle
						if (j < 34) {
							if (NodesArray [i, j + 1].InformerNode.IsObstacle) {
								if (i > 0) {
									if(!NodesArray [i - 1, j + 1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
								if (i < 33) {
									if(!NodesArray [i + 1, j + 1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
							}
						}
						//down obstacle
						if (j > 0) {
							if (NodesArray [i, j -1].InformerNode.IsObstacle) {
								if (i > 0) {
									if(!NodesArray [i - 1, j - 1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
								if (i < 33) {
									if(!NodesArray [i + 1, j - 1].InformerNode.IsObstacle)
										NodesArray [i, j].JumpPoint = true;
								}
							}
						}
					}
				    if (NodesArray[i, j].JumpPoint)
				    {
                        var jPRenderer = NodesArray[i, j].InformerNode.GetComponent<Renderer>();
                        jPRenderer.material.SetColor("_Color", Color.red);
				    }
                  
				}
			}

			//computing distances to jump points and obstacles
			for (var i = 0; i < 34; ++i) {
				for (var j = 0; j < 35; ++j) {
					//Checking up
					var k = 1;
					while (j + k < 35) {
						if(NodesArray [i, j+k].JumpPoint || NodesArray [i, j+k].InformerNode.IsObstacle){
							if(NodesArray [i, j+k].JumpPoint) {
								NodesArray [i, j].NormMatrix[0,1] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[0, 1] = k-1;
							}
							break;
						}
                        if (j + k == 34)
                        {
                            NodesArray[i, j].NormMatrix[0, 1] = k;
                        }
						k++;
					}
					//Checking down
					k = 1;
					while (j-k >= 0) {
						if(NodesArray [i, j-k].JumpPoint || NodesArray [i, j-k].InformerNode.IsObstacle){
							if(NodesArray [i, j-k].JumpPoint) {
                                NodesArray[i, j].NormMatrix[2, 1] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[2, 1] = k-1;
							}
							break;
						}
                        if (j - k == 0)
                        {
                            NodesArray[i, j].NormMatrix[2, 1] = k;
                        }
						k++;
					}
					//Checking right
					k = 1;
					while (i + k < 34) {
						if(NodesArray [i+k, j].JumpPoint || NodesArray [i+k, j].InformerNode.IsObstacle){
							if(NodesArray [i+k, j].JumpPoint) {
                                NodesArray[i, j].NormMatrix[1, 2] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[1, 2] = k-1;
							}
							break;
						}
                        if (i + k == 33)
                        {
                            NodesArray[i, j].NormMatrix[1, 2] = k;
                        }
						k++;
					}
					//Checking left
					k = 1;
					while (i-k >= 0) {
						if(NodesArray [i-k, j].JumpPoint || NodesArray [i-k, j].InformerNode.IsObstacle){
							if(NodesArray [i-k, j].JumpPoint) {
                                NodesArray[i, j].NormMatrix[1, 0] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[1, 0] = k - 1;
							}
							break;
						}
                        if (i - k == 0)
                        {
                            NodesArray[i, j].NormMatrix[1, 0] = k;
                        }
						k++;
					}
					//Checking up-right
					k = 1;
					while (i+k<34 && j+k<35) {
						if(NodesArray [i+k, j+k].JumpPoint || NodesArray [i+k, j+k].InformerNode.IsObstacle){
							if(NodesArray [i+k, j+k].JumpPoint) {
                                NodesArray[i, j].NormMatrix[0, 2] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[0, 2] = k - 1;
							}
							break;
						}
                        if (i + k == 33 || j + k == 34)
                        {
                            NodesArray[i, j].NormMatrix[0, 2] = k;
                        }
						k++;
					}
					//Checking down-right
					k = 1;
					while (i+k<34 && j-k>=0) {
						if(NodesArray [i+k, j-k].JumpPoint || NodesArray [i+k, j-k].InformerNode.IsObstacle){
							if(NodesArray [i+k, j-k].JumpPoint) {
                                NodesArray[i, j].NormMatrix[2, 2] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[2, 2] = k - 1;
							}
							break;
						}
                        if (i + k == 33 || j - k == 0)
                        {
                            NodesArray[i, j].NormMatrix[2, 2] = k;
                        }
						k++;
					}
					//Checking up-left
					k = 1;
					while (i-k>=0 && j+k<35) {
						if(NodesArray [i-k, j+k].JumpPoint || NodesArray [i-k, j+k].InformerNode.IsObstacle){
							if(NodesArray [i-k, j+k].JumpPoint) {
                                NodesArray[i, j].NormMatrix[0, 0] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[0, 0] = k - 1;
							}
							break;
						}
                        if (i - k == 0 || j + k == 34)
                        {
                            NodesArray[i, j].NormMatrix[0, 0] = k;
                        }
						k++;
					}
					//Checking down-left
					k = 1;
					while (i-k>=0 && j-k>=0) {
						if(NodesArray [i-k, j-k].JumpPoint || NodesArray [i-k, j-k].InformerNode.IsObstacle){
							if(NodesArray [i-k, j-k].JumpPoint) {
                                NodesArray[i, j].NormMatrix[2, 0] = k;
							}
							else {
                                NodesArray[i, j].NormMatrix[2, 0] = k - 1;
							}
							break;
						}
					    if (i - k == 0 || j - k == 0)
					    {
                            NodesArray[i, j].NormMatrix[2, 0] = k;
					    }
						k++;
					}
				    if (!NodesArray[i, j].InformerNode.IsObstacle)
				    {
				        var tileText = NodesArray[i, j].InformerNode.GetComponentInChildren<TextMesh>();
				        string text = "";
				        for (int m = 0; m < 3; ++m)
				        {
                            for (int n = 0; n < 3; ++n)
                                text = text + NodesArray[i, j].NormMatrix[m, n].ToString() + " ";
				            text = text + "\n";
				        }

                        tileText.text = text;
				    }
				}
			}
			IsPrecomputed = true;
		}

        public List<Node> JPS(Informer from, Informer to)
        {
            DebugInformationAlgorithm debugInformation;
            var finalPath = JPS(from, to, false, out debugInformation);
            return finalPath;

        }

        public List<Node> JPS(Informer from, Informer to, bool debugFlag, out DebugInformationAlgorithm debugInformation)
		{
            if (from == null || to == null)
            {
                Debug.LogError("Can't run JPS+. Enter proper from and to parameters!");
                debugInformation = null;
                return null;
            }
            //Debug.Log("From: " + from.transform.position);
            //Debug.Log("To: " + to.transform.position);
            if (debugFlag)
            {
                debugInformation = new DebugInformationAlgorithm
                {
                    From = from,
                    To = to,
                    Observed = new List<Node>(),
                    FinalPath = new List<Informer>()
                };
            }
            else
            {
                debugInformation = null;
            }

			if (!IsPrecomputed) {
				Debug.Log ("Precomputing...");
				PrecomputeMap ();
                Debug.Log("Done!");
			}
		    var finish = NodesArray[(int)to.transform.position.x/3, (int)to.transform.position.z/3];
            var linesFromFinish = new StraightLinesFromNode(finish.X(), finish.Y());
            var start = new Tree_Node(null,NodesArray[(int)from.transform.position.x/3, (int)from.transform.position.z/3]);
            Debug.Log("start " + start.Currentnode.InformerNode.transform.position);
            Debug.Log("finish "+finish.InformerNode.transform.position);
            //start.Currentnode.Visited = NodeState.Processed;
		    start.Currentnode = Extensions.IsTargetJP(start.Currentnode, linesFromFinish);
		    var path = new List<Tree_Node>();
		    path.Add(start);
            var current = start;
		    /*if (Extensions.IsTargetJP(start.Currentnode, linesFromFinish).TargetJP)
		    {
                Debug.Log("start is target jp");
                Debug.Log("Destination to finish "+start.Currentnode.DestinationToFinish);
		        finish.DestinationFromPrevious = Extensions.DestinationInverse(start.Currentnode.DestinationToFinish);
		        path.Add(new Tree_Node(start.Currentnode, finish));
		        current = new Tree_Node(start.Currentnode, finish);
		    }
		    var observed = Extensions.Neighbours(start.Currentnode.X(),start.Currentnode.Y(),NodesArray, finish, linesFromFinish);*/
		    var observed = new List<Tree_Node> {current};

		    while (current.Currentnode != finish)
		    {
                if (!observed.Exists(arg => arg.Currentnode.Visited!=NodeState.Processed))
		        {
                    Debug.Log("No path was found");
		            return null;
		        }
                observed[0].Currentnode.Visited = NodeState.Processed;
		        current = observed[0];
                Debug.Log("current " + current.Currentnode.InformerNode.transform.position);
		        observed = observed.OrderBy(arg => arg.Currentnode.Visited).ToList();
		        if (current != start)
		        {
		            while (current.Parent != path[path.Count - 1].Currentnode)
		            {
		                path.RemoveAt(path.Count - 1);
		            }
		        }
		        

		        if (current.Currentnode.TargetJP)
		        {
		            if (Extensions.GoToFinish(current.Currentnode, finish))
		            {
		                path.Add(current);
		                finish.DestinationFromPrevious = current.Currentnode.DestinationToFinish;
                        path.Add(new Tree_Node(current.Currentnode,finish));
		                break;
		            }
                    else continue;
		        }
		        var neighbours = Extensions.Neighbours(current.Currentnode.X(), current.Currentnode.Y(), NodesArray,
		            current.Currentnode.DestinationFromPrevious, finish, linesFromFinish);
                if (current.Currentnode.JumpPoint)
		        {
		            if (neighbours.Count != 0)
		            {
		                observed.InsertRange(0, neighbours);
		                path.Add(current);
		                continue;
		            }
		        }
                var lines = new StraightLinesFromNode(current.Currentnode, Extensions.ToNodes(neighbours));
		        var tempList = new List<Node>();
		        if (lines.Lines != null)
		        {
		            foreach (var line in lines.Lines)
		            {
		                foreach (var lineFromFinish in linesFromFinish.Lines)
		                {
		                    var coordinates = StraightLine.Crossing(line, lineFromFinish);
		                    if (coordinates != null)
		                    {
		                        Debug.Log("Cross point = " + coordinates.X + " " + coordinates.Y);
		                        var tempNode = NodesArray[coordinates.X, coordinates.Y];
		                        tempNode.TargetJP = true;
		                        tempNode.DestinationToFinish = Extensions.DestinationInverse(lineFromFinish.Destination);
		                        Debug.Log("Destination to finish = " + (int) tempNode.DestinationToFinish);
		                        tempNode.Distance = tempNode.InformerNode.Metrics(finish.InformerNode);
		                        tempNode.Visited = NodeState.Discovered;
		                        tempList.Add(tempNode);
		                    }
		                }
		            }
		        }
		        
		        
		        if (tempList.Count != 0)
		        {
		            tempList = tempList.OrderBy(arg => arg.Distance).ToList();
		            foreach (var temp in tempList)
		            {
		                Debug.Log("Cross point "+temp.Position);
		            }
		            var tempTreeList = Tree_Node.NodesToList(tempList, current.Currentnode);
                    path.Add(current);
		            observed.InsertRange(0, tempTreeList);
		        }
		    }
            if(path.Count!=0)
            {
                Debug.Log("Path was found:");
                var finalPath = new List<Node>();
                for (var i = 1; i < path.Count; ++i)
                {
                    var middlePoints = StraightLine.FindMiddlePoints(path[i].Parent, path[i].Currentnode);
                    middlePoints.Remove(new Point(path[i].Parent.X(), path[i].Parent.Y()));
                    finalPath.AddRange(Extensions.ToNodes(middlePoints, NodesArray));
                }

                if (debugInformation != null)
                {
                    debugInformation.Observed = Extensions.ToNodes(
                        observed.Where(arg => arg.Currentnode.Visited==NodeState.Processed).ToList());
                    debugInformation.FinalPath = Extensions.ToInformers(finalPath);
                }

                foreach (var node in finalPath)
                {
                    Debug.Log(node.Position);
                }

                return finalPath;
		    }
            else return null;
		}
    }
}