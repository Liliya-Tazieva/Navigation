using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Map;
using Assets.Scripts.PathFinding;
using Random = System.Random;

public class OnA_StarGUI : MonoBehaviour {
    public List<Texture2D> Maps;
    private int _currentMap ;
    public Informer StartInformer;
    public Informer FinishInformer;
    public GameObject Map;
    private Renderer _startRenderer;
    private Color _startColor;
    private Renderer _finishRenderer;
    private Color _finishColor;
    private bool _jps=false;

    void OnGUI() {
        if (GUI.Button(new Rect(30, 40, 100, 50), "A*")) {
            var controller = GetComponentInChildren<Controller>();
            var nodes = controller.NodesTree
                .Select(kdTreeNode => kdTreeNode.Value)
                .ToList();

            nodes = nodes
                .Where(arg => arg.IsObstacle != true)
                .ToList();

            var rnd = new Random();
            int index1 = 0, index2 = 0;
            while (index1 == index2) {
                index1 = rnd.Next(0, nodes.Count - 1);
                index2 = rnd.Next(0, nodes.Count - 1);
            }

            //Debug.Log("From " + nodes[index1].transform.position);
            //Debug.Log("To " + nodes[index2].transform.position);

            DebugInformationAlgorithm debugInformation;
            controller.AStar(nodes[index1], nodes[index2], controller.Radius, true, out debugInformation);
            controller.InitializeDebugInfo();
            controller.DebugManagerAStar.AddPath(debugInformation);
        }

		if (GUI.Button(new Rect(30, 100, 100, 50), "JPS")) {
			var controller = GetComponentInChildren<Controller>();
			var nodes = controller.NodesTree
				.Select(kdTreeNode => kdTreeNode.Value)
				.ToList();

			nodes = nodes
				.Where(arg => arg.IsObstacle != true)
				.ToList();

			var rnd = new Random();
			int index1 = 0, index2 = 0;
			while (index1 == index2) {
				index1 = rnd.Next(0, nodes.Count - 1);
				index2 = rnd.Next(0, nodes.Count - 1);
			}

            DebugInformationAlgorithm debugInformation;
            controller.JPS(nodes[index1], nodes[index2],true,out debugInformation);
            controller.InitializeDebugInfo();
            controller.DebugManagerAStar.AddPath(debugInformation);
		}

        _jps = GUI.Toggle(new Rect(30, 220, 100, 20), _jps, "Use JPS+");
        if (GUI.Button(new Rect(30, 240, 100, 50), "Run"))
        {
            //Navigation
            var controller = GetComponentInChildren<Controller>();
            //If JPS
            if (_jps)
            {
                DebugInformationAlgorithm debugInformation;
                controller.JPS(StartInformer, FinishInformer, true, out debugInformation);
                controller.InitializeDebugInfo();
                controller.DebugManagerAStar.AddPath(debugInformation);
            }
            //Default algorithm is A_Star
            else
            {
                DebugInformationAlgorithm debugInformation;
                controller.AStar(StartInformer, FinishInformer, controller.Radius, true, out debugInformation);
                controller.InitializeDebugInfo();
                controller.DebugManagerAStar.AddPath(debugInformation);
            }
            //Coloring back
            _startRenderer.material.SetColor("_Color", _startColor);
            _finishRenderer.material.SetColor("_Color", _finishColor);
            StartInformer = null;
            FinishInformer = null;
        }

        if (GUI.Button(new Rect(30, 160, 100, 50), "Next Map")) {
            var mapManager = GetComponentInChildren<MapManager>();
            var controller = mapManager.GetComponent<Controller>();
            controller.NodesTree.Clear();
            if (_currentMap == Maps.Count - 1) _currentMap = 0;
            else ++_currentMap;
            mapManager.Map = Maps[_currentMap];
            var area = mapManager.GetComponentsInChildren<Informer>();
            foreach (var informer in area) {
                Destroy(informer.gameObject);
            }
            for (var i = 0; i < mapManager.Map.height; ++i) {
                for (var j = 0; j < mapManager.Map.width; ++j) {
                    var color = mapManager.Map.GetPixel(i, j);
                    var prefab = mapManager.TilesM.GetPrefab(color);
                    if (prefab == null) {
                        continue;
                    }

                    var position = new Vector3(i*3.0f, 0.0f, j*3.0f);
                    var temp = Instantiate(prefab, position, Quaternion.identity) as GameObject;
                    if (temp != null) {
                        temp.transform.parent = mapManager.gameObject.transform;
                    }
                }
            }
			controller.IsPrecomputed = false;
        }
    }

	// Use this for initialization
	void Start () {
	    _currentMap = 0;
	}

    void Update()
    {
        if (Input.GetButtonDown("Fire1"))
        {
            RaycastHit hit;
            var cam = FindObjectOfType<Camera>();
            Ray ray = cam.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out hit))
            {
                var tile = hit.collider.gameObject;
                if (StartInformer == null)
                {
                    StartInformer = tile.GetComponent<Informer>();
                    _startRenderer = StartInformer.GetComponent<Renderer>();
                    _startColor = _startRenderer.material.GetColor("_Color");
                    _startRenderer.material.SetColor("_Color", Color.cyan);

                    Debug.Log("Start" + StartInformer.transform.position);
                }
                else if (FinishInformer == null)
                {
                    FinishInformer = tile.GetComponent<Informer>();
                    _finishRenderer = FinishInformer.GetComponent<Renderer>();
                    _finishColor = _finishRenderer.material.GetColor("_Color");
                    _finishRenderer.material.SetColor("_Color", Color.magenta);

                    Debug.Log("Finish" + FinishInformer.transform.position);
                }
            }
        }
    }
}
