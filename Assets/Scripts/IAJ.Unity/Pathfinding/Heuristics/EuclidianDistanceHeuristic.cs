using RAIN.Navigation.Graph;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics
{

    public class EuclidianDistanceHeuristic : IHeuristic
    {

        public float H(NavigationGraphNode node, NavigationGraphNode goalNode)
        {
            return Mathf.Sqrt(Mathf.Pow(goalNode.Position.x - node.Position.x, 2) + Mathf.Pow(goalNode.Position.y - node.Position.y, 2));
        }
    }
}
