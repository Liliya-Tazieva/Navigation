using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.PathFinding {
    public class DebugManager : MonoBehaviour {
        private readonly List<DebugInformationAlgorithm> _paths = new List<DebugInformationAlgorithm>();

        public void AddPath(DebugInformationAlgorithm dbi)
        {
            _paths.Add(dbi);
        }

        [UsedImplicitly]
        private void Update() {
            if (_paths.Any()) {
                foreach (var path in _paths) {
                    var component = gameObject.AddComponent<PathRenderer>();
                    component.DebugInformation = path;
                }
                _paths.Clear();
            }
        }
    }
}