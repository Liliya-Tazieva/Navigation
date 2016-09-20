using System.Linq;
using Assets.Scripts.Map;
using UnityEditor;
using UnityEngine;

namespace Assets.Scripts.Editor.Map {
    [CustomEditor(typeof (MapManager))]
    public class MapManagerInspector : UnityEditor.Editor {
        public override void OnInspectorGUI() {
            base.OnInspectorGUI();
            if (GUILayout.Button("ReCreate")) {
                var mapManager = target as MapManager;
                mapManager.transform
                    .Cast<Transform>()
                    .Select(t => t.gameObject)
                    .ToList().ForEach(DestroyImmediate);

                var it = mapManager.InitializeMap();
                while (it.MoveNext()) {}
            }

            if (GUILayout.Button("Destroy")) {
                var mapManager = target as MapManager;
                mapManager.transform
                    .Cast<Transform>()
                    .Select(t => t.gameObject)
                    .ToList().ForEach(DestroyImmediate);
            }
        }
    }
}