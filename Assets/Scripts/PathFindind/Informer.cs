using JetBrains.Annotations;
using UnityEngine;

namespace Assets.Scripts.PathFinding {
    public class Informer : MonoBehaviour {

        [UsedImplicitly]
        public bool IsObstacle;

        [UsedImplicitly]
        public void Start() {
            var parent = transform.parent;
            while (parent != null) {
                var controller = parent.GetComponent<Controller>();
                if (controller != null) {
                    controller.RegisterInformer(this);
                    break;
                }
                parent = parent.transform.parent;
            }
        }
    }
}