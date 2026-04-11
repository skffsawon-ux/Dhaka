import { useEffect, useState } from "react";
import { Plus } from "lucide-react";
import { api } from "../../api";
import RunDetail from "./RunDetail";
import NewRunForm from "./NewRunForm";

interface RunSummary {
  skill_dir_name: string;
  skill_name: string;
  skill_id: string;
  run_id: number;
  status: string;
  daemon_state: string | null;
  error_message: string | null;
  created_at: string | null;
  updated_at: string | null;
  started_at: string | null;
  finished_at: string | null;
  instance_ip: string | null;
  instance_type: string | null;
  training_params: Record<string, unknown> | null;
}

const ACTIVE_STATUSES = [
  "waiting_for_approval",
  "approved",
  "booting",
  "running",
];

export default function TrainingTab() {
  const [runs, setRuns] = useState<RunSummary[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedRun, setSelectedRun] = useState<{
    skill: string;
    runId: number;
  } | null>(null);
  const [showNewRun, setShowNewRun] = useState(false);

  const refresh = () => {
    setLoading(true);
    setError(null);
    api
      .get<RunSummary[]>("/api/training/runs")
      .then(setRuns)
      .catch((e) => setError(e.message))
      .finally(() => setLoading(false));
  };

  useEffect(refresh, []);

  useEffect(() => {
    const hasActive = runs.some((r) => ACTIVE_STATUSES.includes(r.status));
    if (!hasActive) return;
    const interval = setInterval(refresh, 20000);
    return () => clearInterval(interval);
  }, [runs]);

  if (showNewRun) {
    return (
      <NewRunForm
        onBack={() => {
          setShowNewRun(false);
          refresh();
        }}
      />
    );
  }

  if (selectedRun) {
    return (
      <RunDetail
        skillName={selectedRun.skill}
        runId={selectedRun.runId}
        onBack={() => {
          setSelectedRun(null);
          refresh();
        }}
      />
    );
  }

  return (
    <div>
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xs font-semibold text-innate-muted uppercase tracking-wider border-b border-innate-border pb-2 flex-1">
          Training Runs
        </h2>
        <div className="flex gap-2">
          <button
            onClick={() => setShowNewRun(true)}
            className="flex items-center gap-1.5 px-3 py-1.5 rounded-full text-xs font-semibold bg-innate-purple text-white hover:bg-innate-purple-hover transition-colors"
          >
            <Plus size={14} /> New Run
          </button>
          <button
            onClick={refresh}
            className="text-xs text-innate-muted hover:text-black border border-innate-border rounded-full px-3 py-1 transition-all hover:border-black"
          >
            Refresh
          </button>
        </div>
      </div>

      {loading && (
        <p className="text-innate-muted text-sm">Loading runs...</p>
      )}
      {error && (
        <p className="text-innate-orange text-sm">Error: {error}</p>
      )}

      {!loading && runs.length === 0 && (
        <p className="text-innate-muted italic text-sm">
          No training runs found. Submit a skill first, then create a run.
        </p>
      )}

      {!loading && runs.length > 0 && (
        <table className="w-full text-sm border-collapse">
          <thead>
            <tr className="border-b border-innate-border">
              <th className="text-left p-2 text-xs text-innate-muted font-semibold uppercase tracking-wide">
                Skill
              </th>
              <th className="text-left p-2 text-xs text-innate-muted font-semibold uppercase tracking-wide">
                Run
              </th>
              <th className="text-left p-2 text-xs text-innate-muted font-semibold uppercase tracking-wide">
                Status
              </th>
              <th className="text-left p-2 text-xs text-innate-muted font-semibold uppercase tracking-wide">
                Created
              </th>
              <th className="text-left p-2 text-xs text-innate-muted font-semibold uppercase tracking-wide">
                Duration
              </th>
            </tr>
          </thead>
          <tbody>
            {runs.map((r) => (
              <tr
                key={`${r.skill_id}-${r.run_id}`}
                onClick={() =>
                  setSelectedRun({
                    skill: r.skill_dir_name,
                    runId: r.run_id,
                  })
                }
                className="border-b border-innate-border/50 hover:bg-innate-panel cursor-pointer"
              >
                <td className="p-2">
                  <span className="font-medium">{r.skill_name}</span>
                </td>
                <td className="p-2 font-mono text-xs text-gray-600">
                  #{r.run_id}
                </td>
                <td className="p-2">
                  <RunStatusBadge status={r.status} daemon={r.daemon_state} />
                </td>
                <td className="p-2 text-xs text-gray-600">
                  {r.created_at ? fmtTime(r.created_at) : "—"}
                </td>
                <td className="p-2 text-xs text-gray-600">
                  {duration(r.started_at, r.finished_at)}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}

const STATUS_COLORS: Record<string, string> = {
  waiting_for_approval: "bg-innate-orange text-white",
  approved: "bg-innate-purple text-white",
  booting: "bg-innate-purple-hover text-white",
  running: "bg-innate-purple-hover text-white",
  done: "bg-innate-border text-black",
  rejected: "bg-black text-white",
  downloaded: "bg-innate-border text-gray-600",
};

function RunStatusBadge({
  status,
  daemon,
}: {
  status: string;
  daemon: string | null;
}) {
  const colors = STATUS_COLORS[status] || "bg-innate-border text-gray-600";
  const label = status.replace(/_/g, " ") + (daemon ? ` · ${daemon}` : "");
  return (
    <span
      className={`inline-block px-2 py-0.5 rounded-full text-[0.68rem] font-semibold uppercase ${colors}`}
    >
      {label}
    </span>
  );
}

function fmtTime(iso: string): string {
  const d = new Date(iso.endsWith("Z") ? iso : iso + "Z");
  return d.toLocaleString(undefined, {
    month: "short",
    day: "numeric",
    hour: "2-digit",
    minute: "2-digit",
  });
}

function duration(start: string | null, end: string | null): string {
  if (!start) return "—";
  const s = new Date(start.endsWith("Z") ? start : start + "Z");
  const e = end
    ? new Date(end.endsWith("Z") ? end : end + "Z")
    : new Date();
  const secs = Math.floor((e.getTime() - s.getTime()) / 1000);
  if (secs < 60) return `${secs}s`;
  const m = Math.floor(secs / 60);
  if (m < 60) return `${m}m ${secs % 60}s`;
  const h = Math.floor(m / 60);
  return `${h}h ${m % 60}m`;
}
