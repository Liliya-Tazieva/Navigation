using System.Collections.Generic;
using System.Linq;
using Accord.MachineLearning.Structures;
using Assets.Scripts.Core;
using JetBrains.Annotations;
using UnityEngine;

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

        public void RegisterInformer(Informer informer) {
            var position = informer.transform.position;
            NodesTree.Add(position.ToArray(), informer);
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
            DebugInformationAStar debugInformation;
            var finalPath = AStar(from, to, radius, false, out debugInformation);
            return finalPath;
        }

        public List<Informer> AStar(Informer from, Informer to, float radius, bool debugFlag,
            out DebugInformationAStar debugInformation) {
            if (from == null || to == null) {
                Debug.LogError("Can't run A*. Enter proper from and to parameters!");
                debugInformation = null;
                return null;
            }
            //Debug.Log("From: " + from.transform.position);
            //Debug.Log("To: " + to.transform.position);
            if (debugFlag) {
                debugInformation = new DebugInformationAStar {
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
    
        public List<Node> JPS(Informer from, Informer to, float radius) {
            if (from == null || to == null)
            {
                Debug.LogError("Can't run JPS. Enter proper from and to parameters!");
                return null;
            }

			List<Node> open_list = new List<Node>();
			var current = new Node(from, NodeState.Processed);
			open_list.AddRange (NodesTree.Nearest (current.Position().ToArray(), radius).
				ToList().Where (arg => arg.Position() != current.Position()
					&& arg.InformerNode.IsObstacle != true).ToList());
			foreach(var arg in open_list){
				arg.Distance = arg.InformerNode.Metrics(to);
				if (arg.InformerNode.transform.position.x == from.transform.position.x) {
					if (arg.InformerNode.transform.position.z > from.transform.position.z) {
						arg.DestinationToStart = Destination.Up;
					} else {
						arg.DestinationToStart = Destination.Down;
					}
				}
				if (arg.InformerNode.transform.position.z == from.transform.position.z) {
					arg.DestinationToStart = (arg.InformerNode.transform.position.x > from.transform.position.x) 
							? Destination.Right :  Destination.Left;
					
				}
				if (arg.InformerNode.transform.position.x > from.transform.position.x) {
					if (arg.InformerNode.transform.position.z > from.transform.position.z) {
						arg.DestinationToStart = Destination.Up_right;
					} else {
						arg.DestinationToStart = Destination.Down_right;
					}
				}
				if (arg.InformerNode.transform.position.x < from.transform.position.x) {
					if (arg.InformerNode.transform.position.z > from.transform.position.z) {
						arg.DestinationToStart = Destination.Up_left;
					} else {
						arg.DestinationToStart = Destination.Down_left;
					}
				}
			}
			open_list = open_list.OrderBy(arg => arg.Distance).ToList();
			current = open_list [0];
			Debug.Log ("current "+current.InformerNode.transform.position);
			var check_list = new List<Node>{current};
			while (current.InformerNode != to) {
				if (open_list.Count == 1) {
					Debug.Log("No path was found");
					return null;
				}

				var forced_flag = false;
				while (!forced_flag && current != null) {
					var obstacles = NodesTree.Nearest (current.InformerNode.transform.position.ToArray (), radius).
						ToList ().Where (arg => arg.InformerNode.transform.position != current.InformerNode.transform.position
					                && arg.InformerNode.IsObstacle == true).ToList ();
					var neighbours = NodesTree.Nearest (current.InformerNode.transform.position.ToArray (), radius).
						ToList ().Where (arg => arg.InformerNode.transform.position != current.InformerNode.transform.position
							&& arg.InformerNode.IsObstacle != true).ToList ();
					var neighbour = current;
					
					if (current.DestinationToStart == Destination.Up || current.DestinationToStart == Destination.Down) {
						if (obstacles.Exists (arg => arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z)) {
							forced_flag = true;
							open_list.Add (current);
							if (obstacles.Exists (arg => arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x)) {
								if(current.DestinationToStart == Destination.Up){
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x <
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z >
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Up_left;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								} else {
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x <
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z <
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Down_left;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								}						
							}
							if (obstacles.Exists (arg => arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x)) {
								if(current.DestinationToStart == Destination.Up){
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x >
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z >
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Up_right;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								} else {
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x >
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z <
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Down_right;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								}						
							}
						}
						if (current.DestinationToStart == Destination.Up) {
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x &&
							                arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Up;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						} else {
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x &&
							                arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Down;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
					}
					if (current.DestinationToStart == Destination.Left || current.DestinationToStart == Destination.Right) {
						if(obstacles.Exists(arg => arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x)) {
							forced_flag = true;
							open_list.Add (current);
							if (obstacles.Exists (arg => arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z)) {
								if(current.DestinationToStart == Destination.Right){
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x >
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z <
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Down_right;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								} else {
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x <
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z <
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Down_left;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								}						
							}
							if (obstacles.Exists (arg => arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z)) {
								if(current.DestinationToStart == Destination.Right){
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x >
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z >
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Up_right;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
								} else {
									neighbour = neighbours.Find (arg => arg.InformerNode.transform.position.x <
									                current.InformerNode.transform.position.x && arg.InformerNode.transform.position.z >
									                current.InformerNode.transform.position.z);
									if (neighbour != null) {
										neighbour.DestinationToStart = Destination.Up_left;
										neighbour.InformerNode.Metrics (to);
										check_list.Add (neighbour);
									}
									
								}						
							}
						} 
						if (current.DestinationToStart == Destination.Right) {
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							                arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Right;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						} else {
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							                arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Left;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
					}
					if (current.DestinationToStart == Destination.Up_right) {
						if(obstacles.Exists(arg => arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x
								&& arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
							                arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Down_right;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
						if(obstacles.Exists(arg =>arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z
							&& arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find (arg => 
								arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
							                arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Up_left;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Up;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Right;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Up_right;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
					}
					if (current.DestinationToStart == Destination.Up_left) {
						if(obstacles.Exists(arg => arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x
								&& arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Down_left;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						} 
						if (obstacles.Exists(arg =>arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z
							&& arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Up_right;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Up;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Left;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Up_left;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
					}
					if (current.DestinationToStart == Destination.Down_right) {
						if(obstacles.Exists(arg => arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x
								&& arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Up_right;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						} 
						if(obstacles.Exists(arg =>arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z
							&& arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Down_left;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Down;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Right;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Down_right;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
					}
					if (current.DestinationToStart == Destination.Down_left) {
						if(obstacles.Exists(arg => arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x
								&& arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z > current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Up_left;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						} 
						if(obstacles.Exists(arg =>arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z
							&& arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x)) {
							forced_flag = true;
							open_list.Add (current);
							neighbour = neighbours.Find(arg =>  
								arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
								arg.InformerNode.transform.position.x > current.InformerNode.transform.position.x);
							if (neighbour != null) {
								neighbour.DestinationToStart = Destination.Down_right;
								neighbour.InformerNode.Metrics (to);
								check_list.Add (neighbour);
							}
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x == current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Down;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z == current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Left;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
						neighbour = neighbours.Find(arg =>  
							arg.InformerNode.transform.position.z < current.InformerNode.transform.position.z &&
							arg.InformerNode.transform.position.x < current.InformerNode.transform.position.x);
						if (neighbour != null) {
							neighbour.DestinationToStart = Destination.Down_left;
							neighbour.InformerNode.Metrics (to);
							check_list.Add (neighbour);
						}
					}
					if (current.InformerNode == to) {
						forced_flag = true;
						open_list.Add (current);
					}
					check_list.Remove (current);
					check_list = check_list.OrderBy (arg => arg.Distance).ToList();
					current = (check_list.Count==0)
					? null : check_list[0];
				}
				if (current == null) {
					open_list.Remove (current);
					open_list [0].Visited = NodeState.Processed;
					current = open_list	[0];
				}
				check_list.Clear();
			}
			open_list = open_list.Where (arg => arg.Visited == NodeState.Processed).ToList ();
			open_list = Extensions.Inverse_Destination (open_list);
			List<Node> final_path = new List<Node>();
			//////////////////////
			//way back, better to write function in extensions, code is very complicated already
			/////////////////////
			return final_path;
        }
    }
}