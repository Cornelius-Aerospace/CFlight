import plotly.express as px
import plotly.graph_objects as go
import flet as ft
from flet.plotly_chart import PlotlyChart
import time

MAPBOX_TOKEN = "pk.eyJ1IjoibGVhaGNvcm4iLCJhIjoiY2xtcW5pdG83MDQ4NzJrcGl1djByZjA5diJ9.XhEZOCv0_sAj2fiHnyjIDQ"
px.set_mapbox_access_token(MAPBOX_TOKEN)


def gpsMapFigure(latitude, longitude):
    fig = go.Figure(
        go.Scattermapbox(
            lat=[str(latitude)],
            lon=[str(longitude)],
            mode='markers',
            marker=go.scattermapbox.Marker(
                size=10
            ),
            # text=['Montreal'],
        ))

    fig.update_layout(
        hovermode='closest',
        mapbox=dict(
            accesstoken=MAPBOX_TOKEN,
            bearing=0,
            center=go.layout.mapbox.Center(
                lat=latitude,
                lon=longitude
            ),
            pitch=0,
            zoom=15,
        ),
        mapbox_style="dark"

    )
    return fig


def upodateMapFigure(fig, latitude, longitude):
    fig.data[0].lat = [str(latitude)]
    fig.data[0].lon = [str(longitude)]
    fig.update_layout(
        hovermode='closest',
        mapbox=dict(
            accesstoken=MAPBOX_TOKEN,
            bearing=0,
            center=go.layout.mapbox.Center(
                lat=latitude,
                lon=longitude
            ),
            pitch=0,
            zoom=15
        )
    )
    return fig


def main(page: ft.Page):
    lat = 51.236221
    long = -0.570409
    fig = gpsMapFigure(lat, long)
    gpsMap = PlotlyChart(fig, original_size=True)
    page.add(gpsMap)
    while True:
        lat += 0.001
        long += 0.01
        gpsMap.figure = upodateMapFigure(fig, lat, long)
        page.update()
        time.sleep(0.1)


ft.app(target=main)
