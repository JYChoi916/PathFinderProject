using UnityEngine;
using UnityEngine.Tilemaps;

public class PathFinderController : MonoBehaviour
{
    public PathFinder pathFinder;
    public Camera cam;
    public Tilemap pathTileMap;
    public TileBase startTile;
    public TileBase endTile;

    private Vector2Int? startPos = null;
    private Vector2Int? endPos = null;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            // 마우스 클릭 위치
            Vector3 mousePos = cam.ScreenToWorldPoint(Input.mousePosition);
            // 마우스 클릭 위치로부터 타일맵의 셀 위치 가져오기
            Vector3Int cellPos = pathTileMap.WorldToCell(mousePos);
            // 셀 위치를 그리드 위치로 변환
            Vector2Int gridPos = new Vector2Int(cellPos.x, cellPos.y);

            if (!startPos.HasValue)
            {
                // 시작점 설정
                startPos = gridPos;
                pathTileMap.SetTile(new Vector3Int(gridPos.x, gridPos.y, 0), startTile);
            }
            else if (!endPos.HasValue)
            {
                // 종료점 설정
                endPos = gridPos;
                pathTileMap.SetTile(new Vector3Int(gridPos.x, gridPos.y, 0), endTile);
            }
        }
        else if (Input.GetMouseButtonDown(1))
        {
            if (startPos.HasValue)
            {
                // 시작점 제거
                pathTileMap.SetTile(new Vector3Int(startPos.Value.x, startPos.Value.y, 0), null);
            }

            if (endPos.HasValue)
            {
                // 종료점 제거
                pathTileMap.SetTile(new Vector3Int(endPos.Value.x, endPos.Value.y, 0), null);
            }

            pathFinder.ClearPath();

            startPos = null;
            endPos = null;
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            if (startPos.HasValue && endPos.HasValue)
            {
                // 길찾기 시작
                pathFinder.FindPath(startPos.Value, endPos.Value);
            }
        }
    }
}
