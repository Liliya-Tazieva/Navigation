using System.Linq;
using Assets.Scripts.PathFinding;
using UnityEditor;
using UnityEngine;
using Random = System.Random;

namespace Assets.Scripts.Editor.PathFinding {
    [CustomEditor(typeof (Controller))]
    public class ControllerInspector : UnityEditor.Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();

            if (GUILayout.Button("A*")) {
                var controller = (Controller) target;
                var nodes = controller.NodesTree
                    .Select(kdTreeNode => kdTreeNode.Value)
                    .Where(informer => informer)
                    .ToList();

                var from = nodes.Find(x => x.transform.position == controller.From);
                var to = nodes.Find(x => x.transform.position == controller.To);

                DebugInformationAStar debugInformation;
                controller.AStar(from, to, controller.Radius, true, out debugInformation);
                controller.InitializeDebugInfo();
                controller.DebugManagerAStar.AddPath(debugInformation);
            }

            if (GUILayout.Button("Test")) {
                Test();
            }

            if (GUILayout.Button("Test100")) {
                for (var i = 0; i < 100; i++) {
                    Test();
                }
            }

            if (GUILayout.Button("Map update")) {
                var controller = (Controller) target;
                controller.RemoveEmpty();
                PathRenderer.MapRebuild();
            }
        }

        private void Test() {
            var controller = (Controller) target;

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

            DebugInformationAStar debugInformation;
            controller.AStar(nodes[index1], nodes[index2], controller.Radius, true, out debugInformation);
            controller.InitializeDebugInfo();
            controller.DebugManagerAStar.AddPath(debugInformation);
        }
    }
}