import click
import subprocess

@click.command()
@click.option('--fleet-size', type=int, required=True, help='Specify the fleet size')
def fleet_management(fleet_size):
    """
    Allocate and route vehicles based on fleet size.
    """
    click.echo(f'Allocating and routing a fleet of size {fleet_size}')
    try:
        # Run the Action Client CLI internally
        subprocess.run(['python', 'fleet_management_client.py', str(fleet_size)])
    except Exception as e:
        click.echo(f'An error occurred: {e}')
    click.echo('Fleet allocation and routing completed.')

if __name__ == '__main__':
    fleet_management()
