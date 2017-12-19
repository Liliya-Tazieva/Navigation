using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Assets.Scripts.Core;
using Assets.Scripts.Map;
using Assets.Scripts.PathFinding;
using UnityEngine.UI;
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
    private bool _startChanged;

    public void RunJPS()
    {
        //Navigation
        var controller = GetComponentInChildren<Controller>();

        DebugInformationAlgorithm debugInformation;
        controller.JPS(StartInformer, FinishInformer, true, out debugInformation);
        controller.InitializeDebugInfo();
        controller.DebugManagerAStar.AddPath(debugInformation);

        //Coloring back
        _startRenderer.material.SetColor("_Color", _startColor);
        _finishRenderer.material.SetColor("_Color", _finishColor);
        _startChanged = false;
    }

    public void RunAStar()
    {
        //Navigation
        var controller = GetComponentInChildren<Controller>();

        DebugInformationAlgorithm debugInformation;
        controller.AStar(StartInformer, FinishInformer, controller.Radius, true, out debugInformation);
        controller.InitializeDebugInfo();
        controller.DebugManagerAStar.AddPath(debugInformation);

        //Coloring back
        _startRenderer.material.SetColor("_Color", _startColor);
        _finishRenderer.material.SetColor("_Color", _finishColor);
        _startChanged = false;
    }

    public void NextMap()
    {
        var mapManager = GetComponentInChildren<MapManager>();
        var controller = mapManager.GetComponent<Controller>();
        controller.NodesTree.Clear();
        if (_currentMap == Maps.Count - 1) _currentMap = 0;
        else ++_currentMap;
        mapManager.Map = Maps[_currentMap];
        var area = mapManager.GetComponentsInChildren<Informer>();
        foreach (var informer in area)
        {
            Destroy(informer.gameObject);
        }
        for (var i = 0; i < mapManager.Map.height; ++i)
        {
            for (var j = 0; j < mapManager.Map.width; ++j)
            {
                var color = mapManager.Map.GetPixel(i, j);
                var prefab = mapManager.TilesM.GetPrefab(color);
                if (prefab == null)
                {
                    continue;
                }

                var position = new Vector3(i * 3.0f, 0.0f, j * 3.0f);
                var temp = Instantiate(prefab, position, Quaternion.identity) as GameObject;
                if (temp != null)
                {
                    temp.transform.parent = mapManager.gameObject.transform;
                }
            }
        }
        controller.IsPrecomputed = false;
    }

    public void JPSTest()
    {
        var controller = GetComponentInChildren<Controller>();
        var nodes = controller.NodesTree
            .Select(kdTreeNode => kdTreeNode.Value)
            .ToList();

        nodes = nodes
            .Where(arg => arg.IsObstacle != true)
            .ToList();

        var rnd = new Random();
        int index1 = 0, index2 = 0;
        while (index1 == index2)
        {
            index1 = rnd.Next(0, nodes.Count - 1);
            index2 = rnd.Next(0, nodes.Count - 1);
        }

        ColorStartAndFinish(nodes[index1], nodes[index2]);

        DebugInformationAlgorithm debugInformation;
        controller.JPS(StartInformer, FinishInformer, true, out debugInformation);
        controller.InitializeDebugInfo();
        controller.DebugManagerAStar.AddPath(debugInformation);

        //Coloring back
        _startRenderer.material.SetColor("_Color", _startColor);
        _finishRenderer.material.SetColor("_Color", _finishColor);
    }

    public void AStarTest()
    {
        var controller = GetComponentInChildren<Controller>();
        var nodes = controller.NodesTree
            .Select(kdTreeNode => kdTreeNode.Value)
            .ToList();

        nodes = nodes
            .Where(arg => arg.IsObstacle != true)
            .ToList();

        var rnd = new Random();
        int index1 = 0, index2 = 0;
        while (index1 == index2)
        {
            index1 = rnd.Next(0, nodes.Count - 1);
            index2 = rnd.Next(0, nodes.Count - 1);
        }

        ColorStartAndFinish(nodes[index1], nodes[index2]);

        DebugInformationAlgorithm debugInformation;
        controller.AStar(StartInformer, FinishInformer, controller.Radius, true, out debugInformation);
        controller.InitializeDebugInfo();
        controller.DebugManagerAStar.AddPath(debugInformation);

        //Coloring back
        _startRenderer.material.SetColor("_Color", _startColor);
        _finishRenderer.material.SetColor("_Color", _finishColor);
    }

    private void ColorStartAndFinish(Informer start, Informer finish)
    {
        if (_startRenderer != null)
        {
            _startRenderer.material.SetColor("_Color", _startColor);
        }

        StartInformer = start;
        _startRenderer = StartInformer.GetComponent<Renderer>();
        _startColor = _startRenderer.material.GetColor("_Color");
        _startRenderer.material.SetColor("_Color", Color.cyan);

        Debug.Log("Start" + StartInformer.transform.position);

        if (_finishRenderer != null)
        {
            _finishRenderer.material.SetColor("_Color", _finishColor);
        }

        FinishInformer = finish;
        _finishRenderer = FinishInformer.GetComponent<Renderer>();
        _finishColor = _finishRenderer.material.GetColor("_Color");
        _finishRenderer.material.SetColor("_Color", Color.magenta);

        Debug.Log("Finish" + FinishInformer.transform.position);
    }

    public void PrecomputeMap()
    {
        var controller = GetComponentInChildren<Controller>();
        if(!controller.IsPrecomputed) controller.PrecomputeMap();
    }

    public void GoalBounding()
    {
        var controller = GetComponentInChildren<Controller>();
        if (!controller.IsPrecomputed) controller.PrecomputeMap();
        controller.ComputeBoundingBoxes();
    }

    public void ClearMap()
    {
        var colorList = GetComponent<DefaultColours>();
        if (colorList.DefaultMaterials.Count == 0) return;
        var controller = GetComponentInChildren<Controller>();
        var nodesArray = controller.NodesArray;
        for (var j = 0; j < Maps[_currentMap].height; ++j)
        {
            for (var k = 0; k < Maps[_currentMap].width; ++k)
            {
                if (!nodesArray[j, k].InformerNode.IsObstacle)
                {
                    var currentRenderer = nodesArray[j,k].InformerNode.GetComponent<Renderer>();
                    currentRenderer.material = colorList.DefaultMaterials[0];
                }
            }
        }
        Extensions.ShowJP(controller.JumpPoints);
        _startRenderer.material.SetColor("_Color", Color.cyan);
        _finishRenderer.material.SetColor("_Color", Color.magenta);
    }

    public void ShowFinalPathAStar()
    {
        if (StartInformer != null && FinishInformer != null)
        {
            var controller = GetComponentInChildren<Controller>();
            var path = controller.AStar(StartInformer, FinishInformer, controller.Radius);
            var debugInfo = new DebugInformationAlgorithm
            {
                From = StartInformer,
                To = FinishInformer,
                FinalPath = path,
                Destroy = false
            };
            controller.InitializeDebugInfo();
            controller.DebugManagerAStar.AddPath(debugInfo);
        }
        else Debug.LogError("Enter proper arguments");
    }

    public void ShowFinalPathJPS()
    {
        if (StartInformer != null && FinishInformer != null)
        {
            var controller = GetComponentInChildren<Controller>();
            var path = controller.JPS(StartInformer, FinishInformer);
            var debugInfo = new DebugInformationAlgorithm
            {
                From = StartInformer,
                To = FinishInformer,
                FinalPathJPS = path,
                Destroy = false
            };
            controller.InitializeDebugInfo();
            controller.DebugManagerAStar.AddPath(debugInfo);
        }
        else Debug.LogError("Enter proper arguments");
    }

    public void ShowNeighboursAndTJP()
    {
        if (StartInformer != null && FinishInformer != null)
        {
            var controller = GetComponentInChildren<Controller>();
            if(!controller.IsPrecomputed) controller.PrecomputeMap();
            DebugInformationAlgorithm debugInformation;
            controller.InitializeDebugInfo();
            Extensions.NeighboursAndTJP(StartInformer,FinishInformer,controller.NodesArray,out debugInformation);
            controller.DebugManagerAStar.AddPath(debugInformation);

        }
        else Debug.LogError("Enter proper arguments");
    }

    // Use this for initialization
	void Start () {
	    _currentMap = 0;
	    _startChanged = false;
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
                if (!_startChanged)
                {
                    _startChanged = true;
                    if (_startRenderer != null)
                    {
                        _startRenderer.material.SetColor("_Color", _startColor);
                    }

                    StartInformer = tile.GetComponent<Informer>();
                    _startRenderer = StartInformer.GetComponent<Renderer>();
                    _startColor = _startRenderer.material.GetColor("_Color");
                    _startRenderer.material.SetColor("_Color", Color.cyan);

                    Debug.Log("Start" + StartInformer.transform.position);
                }
                else
                {
                    _startChanged = false;
                    if (_finishRenderer != null)
                    {
                        _finishRenderer.material.SetColor("_Color", _finishColor);
                    }

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
