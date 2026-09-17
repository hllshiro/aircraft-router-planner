import { Component, type ReactNode } from 'react';

interface Props {
  children: ReactNode;
}

interface State {
  error: Error | null;
}

/** 渲染错误捕获：显示错误信息而非白屏（开发期定位用，也提升 Demo 健壮性） */
export class ErrorBoundary extends Component<Props, State> {
  state: State = { error: null };

  static getDerivedStateFromError(error: Error): State {
    return { error };
  }

  render() {
    if (this.state.error) {
      return (
        <div
          style={{
            position: 'absolute',
            inset: 0,
            zIndex: 999,
            background: '#1a1a2e',
            color: '#ff6b6b',
            padding: 20,
            fontFamily: 'monospace',
            fontSize: 13,
            whiteSpace: 'pre-wrap',
            overflow: 'auto',
          }}
        >
          <h2>⚠ 渲染错误</h2>
          <div>{this.state.error.message}</div>
          <pre style={{ marginTop: 12 }}>{this.state.error.stack}</pre>
          <button
            style={{ marginTop: 16 }}
            onClick={() => this.setState({ error: null })}
          >
            重试
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}
