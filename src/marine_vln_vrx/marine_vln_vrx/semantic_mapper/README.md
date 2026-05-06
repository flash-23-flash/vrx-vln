# semantic_mapper

职责：
- 订阅 `/vln/scene_state`
- 维护短时对象缓存（带 TTL）
- 发布语义地图 `/vln/semantic_map`

参数：
- `scene_topic`
- `semantic_map_topic`
- `cache_ttl_sec`
- `publish_rate_hz`
- `log_root`
