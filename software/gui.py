import flet as ft

connectionStatus = True
FCErrors = False
gpsFix = True
gpsError = False

gpsSatiliteCount = 6
altitude = 0.8
pressure = 1000.24
aglPressure = 1000.27
gpsAltitude = 0.6

latitude = 51.5
longitude = -50.2

googleMapsApiKey = "AIzaSyDMoUsw_OxCRCZq-LQ9-PAGnLmPhe_LB-c"

def main(page: ft.Page):
    page.title = "CFlight"
    page.vertical_alignment = ft.MainAxisAlignment.CENTER
    page.bgcolor = ft.colors.BLACK

    rowAItems = [
        ft.Container(
            content=ft.Column(
                [
                    ft.Text(value="Connection Status", size=20,
                            weight=ft.FontWeight.BOLD, color=ft.colors.WHITE),
                    ft.Text(value="Disconnected" if not connectionStatus else "Connected", size=25, color=ft.colors.WHITE)]),
            alignment=ft.alignment.center,
            width=200,
            height=100,
            bgcolor=ft.colors.GREEN if connectionStatus else ft.colors.RED,
            border_radius=ft.border_radius.all(5),
            col=4,
        ),
        ft.Container(
            content=ft.Column(
                [
                    ft.Text(value="FC Status", size=20,
                            weight=ft.FontWeight.BOLD, color=ft.colors.WHITE),
                    ft.Text(value="Nominal" if not FCErrors else "Errors", size=25, color=ft.colors.WHITE)]),
            alignment=ft.alignment.center,
            width=200,
            height=100,
            bgcolor=ft.colors.GREEN if not FCErrors else ft.colors.RED,
            border_radius=ft.border_radius.all(5),
            col=4,
        ),
        ft.Container(
            content=ft.Column(
                [
                    ft.Text(value="GPS Status", size=20,
                            weight=ft.FontWeight.BOLD, color=ft.colors.WHITE),
                    ft.Text(value="Nominal" if gpsFix else "Connecting",
                            size=25, color=ft.colors.WHITE),
                    ft.Text(value="{} Sat in view".format(gpsSatiliteCount),
                            size=15, color=ft.colors.WHITE,
                            )], spacing=2),
            alignment=ft.alignment.center,
            width=200,
            height=100,
            bgcolor=ft.colors.GREEN if gpsFix else ft.colors.AMBER,
            border_radius=ft.border_radius.all(5),
            col=4,
        ),
    ]
    rowBItems = [
        ft.Container(content=ft.Column([
            ft.Text(value="Altitude: {} m".format(altitude), size=20,
                    weight=ft.FontWeight.BOLD, color=ft.colors.WHITE),
            ft.Text(value="Pressure: {} MPa".format(pressure), size=17,
                    color=ft.colors.WHITE),
            ft.Text(value="AGL Pressure: {} MPa".format(aglPressure),
                    size=15, color=ft.colors.WHITE)
        ],
            spacing=2),
            alignment=ft.alignment.center,
            width=200,
            height=100,
            bgcolor=ft.colors.BLUE,
            border_radius=ft.border_radius.all(5),
            col=6),
        ft.VerticalDivider(),
        ft.Container(content=ft.Column([
            ft.Text(value="GPS alt: {} m".format(gpsAltitude), size=20,
                    weight=ft.FontWeight.BOLD, color=ft.colors.WHITE),
            ft.Text(value="Latitude: {}".format(latitude), size=20,
                    color=ft.colors.WHITE),
            ft.Text(value="Longitude: {}".format(longitude),
                    size=20, color=ft.colors.WHITE)
        ],
            spacing=2),
            alignment=ft.alignment.center,
            width=200,
            height=100,
            bgcolor=ft.colors.BLUE_GREY,
            border_radius=ft.border_radius.all(5),
            col=6),
    ]
    rowCItems = []
    rowA = ft.ResponsiveRow(spacing=10, controls=rowAItems)
    rowB = ft.Row(spacing=20, controls=rowBItems)
    rowC = ft.Row(spacing=30, controls=rowCItems)

    page.add(ft.Column([rowA, ft.Divider(), rowB, rowC]))


ft.app(target=main)
