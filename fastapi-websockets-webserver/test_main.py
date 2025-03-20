"""test_main file"""

from fastapi.testclient import TestClient

from main import app

client = TestClient(app)

def test_health_route():
    """test health route"""
    response = client.get("/api/health")
    assert response.status_code == 200
    assert response.json() == {"status": "OK"}
