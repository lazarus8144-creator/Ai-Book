#!/usr/bin/env python3
"""
Test Claude Subagent Enhancement
=================================

Quick demo script to test subagent-enhanced RAG responses.

Usage:
    # Basic test
    python scripts/test_subagent.py

    # Test specific question
    python scripts/test_subagent.py --question "What is ROS 2?"

    # Compare with/without subagent
    python scripts/test_subagent.py --compare

Requirements:
    - Backend server must be running (uvicorn app.main:app)
    - ANTHROPIC_API_KEY must be set in .env
    - ENABLE_SUBAGENT_ENHANCEMENT=true in .env
"""

import argparse
import asyncio
import sys
from pathlib import Path
from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown
from rich.table import Table

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.rag_pipeline_enhanced import EnhancedRAGPipeline


console = Console()


async def test_single_query(question: str, enable_subagent: bool = True):
    """Test a single query with optional subagent enhancement"""

    console.print(f"\n{'='*70}")
    console.print(f"[bold cyan]Testing RAG Query{' with Subagent' if enable_subagent else ' (baseline)'}[/bold cyan]")
    console.print(f"{'='*70}\n")

    console.print(f"[yellow]Question:[/yellow] {question}\n")

    # Initialize pipeline
    pipeline = EnhancedRAGPipeline(enable_subagent=enable_subagent)

    # Query
    console.print("⏳ Processing query...")
    result = await pipeline.query(question)

    # Display answer
    console.print("\n[green]Answer:[/green]")
    console.print(Panel(Markdown(result.answer), border_style="green"))

    # Display metadata
    table = Table(title="Query Metadata", show_header=True)
    table.add_column("Metric", style="cyan")
    table.add_column("Value", style="magenta")

    table.add_row("Response Time", f"{result.response_time_ms}ms")
    table.add_row("Tokens Used", str(result.tokens_used))
    table.add_row("Sources Found", str(len(result.sources)))

    if hasattr(result, 'metadata') and result.metadata.get('enhanced_by_subagent'):
        table.add_row("Enhanced by Subagent", "✅ Yes")
        table.add_row("Accuracy Score", f"{result.metadata.get('accuracy_score')}/100")
        table.add_row("Has Code Example", "✅ Yes" if result.metadata.get('has_code_example') else "❌ No")
        table.add_row("Enhancement Time", f"{result.metadata.get('enhancement_time_ms')}ms")
    else:
        table.add_row("Enhanced by Subagent", "❌ No")

    console.print("\n")
    console.print(table)

    # Display sources
    if result.sources:
        console.print("\n[blue]Sources:[/blue]")
        for i, source in enumerate(result.sources, 1):
            console.print(f"  {i}. {source.module} - {source.title}")
            console.print(f"     Score: {source.score:.2f} | {source.url}")

    # Display follow-up questions if available
    if hasattr(result, 'metadata') and result.metadata.get('follow_up_questions'):
        console.print("\n[magenta]Suggested Follow-up Questions:[/magenta]")
        for i, q in enumerate(result.metadata['follow_up_questions'], 1):
            console.print(f"  {i}. {q}")

    console.print("\n")


async def compare_with_without_subagent(question: str):
    """Compare RAG responses with and without subagent enhancement"""

    console.print(f"\n{'='*70}")
    console.print("[bold cyan]Comparing Baseline vs Subagent-Enhanced RAG[/bold cyan]")
    console.print(f"{'='*70}\n")

    console.print(f"[yellow]Question:[/yellow] {question}\n")

    # Test without subagent
    console.print("[dim]Testing baseline RAG (no subagent)...[/dim]")
    pipeline_baseline = EnhancedRAGPipeline(enable_subagent=False)
    result_baseline = await pipeline_baseline.query(question)

    # Test with subagent
    console.print("[dim]Testing enhanced RAG (with subagent)...[/dim]\n")
    pipeline_enhanced = EnhancedRAGPipeline(enable_subagent=True)
    result_enhanced = await pipeline_enhanced.query(question)

    # Display comparison
    console.print("\n[bold]BASELINE ANSWER (No Subagent):[/bold]")
    console.print(Panel(result_baseline.answer, border_style="yellow"))

    console.print("\n[bold]ENHANCED ANSWER (With Subagent):[/bold]")
    console.print(Panel(result_enhanced.answer, border_style="green"))

    # Comparison table
    table = Table(title="Performance Comparison", show_header=True)
    table.add_column("Metric", style="cyan")
    table.add_column("Baseline", style="yellow")
    table.add_column("Enhanced", style="green")

    table.add_row(
        "Response Time",
        f"{result_baseline.response_time_ms}ms",
        f"{result_enhanced.response_time_ms}ms"
    )
    table.add_row(
        "Tokens Used",
        str(result_baseline.tokens_used),
        str(result_enhanced.tokens_used)
    )
    table.add_row(
        "Answer Length",
        f"{len(result_baseline.answer)} chars",
        f"{len(result_enhanced.answer)} chars"
    )

    if hasattr(result_enhanced, 'metadata') and result_enhanced.metadata.get('enhanced_by_subagent'):
        table.add_row(
            "Accuracy Score",
            "N/A",
            f"{result_enhanced.metadata.get('accuracy_score')}/100"
        )
        table.add_row(
            "Code Example",
            "N/A",
            "✅ Yes" if result_enhanced.metadata.get('has_code_example') else "❌ No"
        )

    console.print("\n")
    console.print(table)
    console.print("\n")


async def run_test_suite():
    """Run a comprehensive test suite"""

    console.print(Panel.fit(
        "[bold cyan]Claude Subagent Test Suite[/bold cyan]\n"
        "Testing RAG enhancement with Claude Sonnet 4.5",
        border_style="cyan"
    ))

    test_questions = [
        "What is ROS 2?",
        "How do I create a ROS 2 node in Python?",
        "Explain the difference between Gazebo and Unity for robotics simulation",
        "What are VLA models used for in robotics?",
    ]

    for i, question in enumerate(test_questions, 1):
        console.print(f"\n[bold]Test {i}/{len(test_questions)}[/bold]")
        await test_single_query(question, enable_subagent=True)

        if i < len(test_questions):
            console.print("[dim]Press Enter to continue...[/dim]")
            input()


async def main():
    parser = argparse.ArgumentParser(
        description="Test Claude Subagent enhancement for RAG chatbot",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        '--question',
        type=str,
        help='Specific question to test'
    )

    parser.add_argument(
        '--compare',
        action='store_true',
        help='Compare baseline vs enhanced RAG'
    )

    parser.add_argument(
        '--suite',
        action='store_true',
        help='Run full test suite'
    )

    args = parser.parse_args()

    try:
        if args.suite:
            await run_test_suite()
        elif args.compare:
            question = args.question or "What is ROS 2?"
            await compare_with_without_subagent(question)
        elif args.question:
            await test_single_query(args.question, enable_subagent=True)
        else:
            # Default: single test with default question
            await test_single_query("What is ROS 2?", enable_subagent=True)

        console.print("\n[green]✅ Test completed successfully![/green]\n")

    except KeyboardInterrupt:
        console.print("\n[yellow]⚠️  Test interrupted by user[/yellow]")
        sys.exit(1)
    except Exception as e:
        console.print(f"\n[red]❌ Error: {e}[/red]")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
