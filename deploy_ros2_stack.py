#!/usr/bin/env python3
"""Deploy ROS2 stack to LanderPi robot."""

import json
import typer
from pathlib import Path
from fabric import Connection
from rich.console import Console
from rich.panel import Panel

app = typer.Typer()
console = Console()


def get_config():
    """Load connection config from config.json."""
    config_path = Path(__file__).parent / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            return json.load(f)
    return {}


def upload_directory(conn: Connection, local_path: Path, remote_path: str, home_dir: str):
    """Upload a directory recursively."""
    # Expand ~ to actual home directory
    if remote_path.startswith("~"):
        remote_path = remote_path.replace("~", home_dir, 1)
    conn.run(f"mkdir -p {remote_path}")
    for item in local_path.rglob("*"):
        if item.is_file() and "__pycache__" not in str(item):
            rel_path = item.relative_to(local_path)
            remote_file = f"{remote_path}/{rel_path}"
            remote_dir = str(Path(remote_file).parent)
            conn.run(f"mkdir -p {remote_dir}")
            conn.put(str(item), remote_file)
            console.print(f"  [dim]{rel_path}[/dim]")


@app.command()
def deploy(
    host: str = typer.Option(None, help="Robot IP address"),
    user: str = typer.Option(None, help="SSH username"),
    password: str = typer.Option(None, help="SSH password"),
    start: bool = typer.Option(True, help="Start the stack after deployment"),
):
    """Deploy ROS2 stack to LanderPi robot."""

    # Load defaults from config.json
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not all([host, user, password]):
        console.print("[red]Missing connection details. Use --host, --user, --password or config.json[/red]")
        raise typer.Exit(1)

    console.print(Panel(f"Deploying ROS2 stack to {user}@{host}", title="LanderPi Deploy"))

    base_path = Path(__file__).parent

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        # Get home directory for path expansion
        result = conn.run("echo $HOME", hide=True)
        home_dir = result.stdout.strip()

        # Create remote directories
        console.print("\n[bold]Creating directories...[/bold]")
        conn.run("mkdir -p ~/landerpi/{ros2_nodes,docker,config,drivers}")
        # Create sensor bridge data directory (shared with Docker container)
        conn.run("mkdir -p ~/landerpi_data")
        console.print("  [dim]Created ~/landerpi_data for sensor bridge[/dim]")

        # Upload ros2_nodes
        console.print("\n[bold]Uploading ros2_nodes/...[/bold]")
        upload_directory(conn, base_path / "ros2_nodes", "~/landerpi/ros2_nodes", home_dir)

        # Upload docker
        console.print("\n[bold]Uploading docker/...[/bold]")
        upload_directory(conn, base_path / "docker", "~/landerpi/docker", home_dir)

        # Upload config
        console.print("\n[bold]Uploading config/...[/bold]")
        upload_directory(conn, base_path / "config", "~/landerpi/config", home_dir)

        # Upload drivers (read-only reference)
        console.print("\n[bold]Uploading drivers/ros_robot_controller-ros2/...[/bold]")
        upload_directory(
            conn,
            base_path / "drivers" / "ros_robot_controller-ros2",
            "~/landerpi/drivers/ros_robot_controller-ros2",
            home_dir
        )

        if start:
            console.print("\n[bold]Restarting ROS2 stack...[/bold]")
            # --force-recreate ensures container restarts with new config
            result = conn.run(
                "cd ~/landerpi/docker && docker compose up -d --build --force-recreate",
                warn=True
            )
            if result.ok:
                console.print("[green]ROS2 stack started successfully![/green]")
            else:
                console.print("[red]Failed to start stack. Check logs with:[/red]")
                console.print("  docker logs landerpi-ros2")

        # Verify
        console.print("\n[bold]Verifying deployment...[/bold]")
        result = conn.run("docker ps --filter name=landerpi-ros2 --format '{{.Status}}'", warn=True)
        if "Up" in result.stdout:
            console.print("[green]Container is running![/green]")
            console.print("\nTest with:")
            console.print("  uv run python test_chassis_motion.py test")
        else:
            console.print("[yellow]Container not running yet. Check logs.[/yellow]")


@app.command()
def stop(
    host: str = typer.Option(None),
    user: str = typer.Option(None),
    password: str = typer.Option(None),
):
    """Stop the ROS2 stack on robot."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        conn.run("cd ~/landerpi/docker && docker compose down")
        console.print("[green]ROS2 stack stopped[/green]")


@app.command()
def logs(
    host: str = typer.Option(None),
    user: str = typer.Option(None),
    password: str = typer.Option(None),
    follow: bool = typer.Option(False, "-f", help="Follow log output"),
):
    """View ROS2 stack logs."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        cmd = "docker logs landerpi-ros2"
        if follow:
            cmd += " -f"
        conn.run(cmd)


@app.command()
def rebuild(
    host: str = typer.Option(None),
    user: str = typer.Option(None),
    password: str = typer.Option(None),
):
    """Rebuild Docker image from scratch (includes Hailo support)."""
    config = get_config()
    host = host or config.get("host")
    user = user or config.get("user")
    password = password or config.get("password")

    if not all([host, user, password]):
        console.print("[red]Missing connection details.[/red]")
        raise typer.Exit(1)

    console.print(Panel(
        f"Rebuilding Docker image on {host}\n\n"
        "This will:\n"
        "1. Stop current stack\n"
        "2. Upload latest Dockerfile\n"
        "3. Rebuild image with Hailo support\n"
        "4. Start stack\n\n"
        "[yellow]This takes 5-10 minutes[/yellow]",
        title="Docker Rebuild"
    ))

    base_path = Path(__file__).parent

    with Connection(host, user=user, connect_kwargs={"password": password}) as conn:
        result = conn.run("echo $HOME", hide=True)
        home_dir = result.stdout.strip()

        # Stop current stack
        console.print("\n[bold]Stopping current stack...[/bold]")
        conn.run("cd ~/landerpi/docker && docker compose down", warn=True)

        # Upload docker directory (includes new Dockerfile)
        console.print("\n[bold]Uploading docker/...[/bold]")
        upload_directory(conn, base_path / "docker", "~/landerpi/docker", home_dir)

        # Rebuild image with no cache
        console.print("\n[bold]Building Docker image (this takes a while)...[/bold]")
        result = conn.run(
            "cd ~/landerpi/docker && docker compose build --no-cache",
            warn=True
        )

        if not result.ok:
            console.print("[red]Build failed! Check output above.[/red]")
            raise typer.Exit(1)

        console.print("[green]Image built successfully![/green]")

        # Start stack
        console.print("\n[bold]Starting stack...[/bold]")
        result = conn.run(
            "cd ~/landerpi/docker && docker compose up -d",
            warn=True
        )

        if result.ok:
            console.print("[green]Stack started![/green]")
            console.print("\nTest Hailo with:")
            console.print("  uv run python deploy_explorer.py start --yolo-hailo --duration 2")
        else:
            console.print("[red]Failed to start stack[/red]")


if __name__ == "__main__":
    app()
