from benchmark_runner import build_compare_arg_parser, run_benchmark


def main():
    args = build_compare_arg_parser().parse_args()
    run_benchmark(
        output_dir=args.output_dir,
        show=args.show,
        make_animation=args.animate,
        show_animation=args.show_animation,
    )


if __name__ == "__main__":
    main()
