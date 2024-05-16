/*
 *	ultra_r5 module implementation
 *
 *	This file is part of OTAWA
 *	Copyright (c) 2017, IRIT UPS.
 *
 *	OTAWA is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	OTAWA is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with OTAWA; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <otawa/events/StandardEventBuilder.h>
#include <otawa/etime/EdgeTimeBuilder.h>
#include <otawa/prop/DynIdentifier.h>
#include <elm/sys/Path.h>
#include <otawa/loader/arm.h>
#include <elm/io/FileOutput.h>
#include <elm/data/Vector.h>

typedef enum {
	NOT_PREDICTED = 0x1, // The processor is always making a not-take prediction. If the instruction is unconditional or if it passes
						// its condition code, the pipeline is flushed, add eight to the Cycles.
	PREDICTED = 0x2, // The processor predicts the instruction. 
					// No performance penalty occurs on a correct prediction
	STOP = 0x4, // The performance penalty is similar to the Not Predicted case,
				// therefore, add eight to the Cycles.
	NOT_APPLICABLE = 0x8 
} branch_behavior_t;
 
typedef struct {
	int ex_cost;
	branch_behavior_t br_behavior;
	bool unknown;
} r52f_time_t;

r52f_time_t R52F_time_int_single_cyle = {1, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_single_cyle_PC = {1, STOP, false};
r52f_time_t R52F_time_int_branch = {1, PREDICTED, false};
r52f_time_t R52F_time_int_ld = {3, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_ldm = {8, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_ld_to_pc = {3, STOP, false};
r52f_time_t R52F_time_int_st_to_pc = {3, STOP, false};
r52f_time_t R52F_time_int_stm = {8, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_st = {3, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_msr = {5, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_pop = {6, PREDICTED, false};
r52f_time_t R52F_time_int_push = {6, PREDICTED, false};
r52f_time_t R52F_time_int_lsx = {1, NOT_PREDICTED, false};
r52f_time_t R52F_time_int_MCR = {6, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_pldx = {3, NOT_APPLICABLE, false};
r52f_time_t R52F_time_int_s_udiv = {9, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_ldm = {16, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_single_cyle = {1, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_ld = {4, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_pop_push = {16, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_stm = {16, NOT_PREDICTED, false};
r52f_time_t R52F_time_fpu_simd_st = {5, NOT_PREDICTED, false};
r52f_time_t R52F_time_unknown = {10, STOP, true};
#include "armCortexR52F_time.h"

namespace otawa { namespace ultra {
    using namespace elm::io;
	extern p::id<bool> WRITE_LOG;
	
    typedef enum {
        FE    = 0,
        DE    = 1,
        ISS   = 2,
        EXE   = 3,
        WR    = 4,
        CNT   = 5
    } pipeline_stage_t;

	
	class R52FExeGraph: public etime::EdgeTimeGraph {
	public:
		
		R52FExeGraph(WorkSpace* ws,
                 ParExeProc* proc, 
                 Vector<Resource* >* hw_resources, 
				 ParExeSequence* seq,
                 const PropList &props,
                 FileOutput* out, 
                 elm::Vector<Address>* unknown_inst_address) : etime::EdgeTimeGraph(ws, proc, hw_resources, seq, props), 
                                                                exec_int(0), exec_fpu_simd(0), _out(out), 
                                                                _unknown_inst_address(unknown_inst_address) {
			
			// Try to find arm loader with arm information
			DynIdentifier<arm::Info* > id("otawa::arm::Info::ID");
			info = id(_ws->process());
			if (!info)
				throw Exception("ARM loader with otawa::arm::INFO is required !");
			// Get memory configuration
			mem = hard::MEMORY_FEATURE.get(ws);
			ASSERTP(mem, "Memory feature not found");
		}
		
		void addEdgesForPipelineOrder() override {
			ParExeGraph::addEdgesForPipelineOrder();
			// Add latency penalty to Exec-FU nodes
			for (InstIterator inst(this); inst(); inst++) {
				// get cycle_time_info of inst
				r52f_time_t* inst_cycle_timing = getInstCycleTiming(inst->inst());
				int cost = inst_cycle_timing->ex_cost;
				if (inst->inst()->isMulti())
					cost = (inst->inst()->multiCount() > 3) ? inst->inst()->multiCount() : 3;
				if (inst_cycle_timing->br_behavior & (STOP|NOT_PREDICTED))
					cost += 8;
				if (cost > 1) {
                    inst->firstFUNode()->setLatency(cost / 2);
					inst->lastFUNode()->setLatency(cost - (cost / 2));
				} else
				    inst->lastFUNode()->setLatency(0);
			}
		}
		
		void addEdgesForMemoryOrder() override {

			// The datasheet does not give enough information about consecutive ld/st issuing.
			// Although it explains that in some cases, two or more consecutive mem-instructions can not be dual-issued. 
			// Here we consider the worst-case situation.
			
			// Call the default implementation
			ParExeGraph::addEdgesForMemoryOrder();
			
			static string memory_order = "memory order";
			auto stage = _microprocessor->execStage();

			// looking in turn each FU
			for (int i=0 ; i<stage->numFus() ; i++) {
				ParExeStage* fu_stage = stage->fu(i)->firstStage();
				ParExeNode* previous_load = nullptr;

				// look for each node of this FU
				for (int j=0 ; j<fu_stage->numNodes() ; j++){
					ParExeNode* node = fu_stage->node(j);
					// found a load instruction
					if (node->inst()->inst()->isLoad()) {
						// if any, add dependency on the previous load
						if (previous_load)
							new ParExeEdge(previous_load, node, ParExeEdge::SOLID, 0, memory_order);
						
						// current node becomes the new previous load
						for (InstNodeIterator last_node(node->inst()); last_node() ; last_node++)
							if (last_node->stage()->category() == ParExeStage::FU)
								previous_load = *last_node;
					}
				}
			}
		}
		
		void addEdgesForDataDependencies() override {
			string data_dep("");
			ParExeStage* exec_stage = _microprocessor->execStage();
			// for each functional unit
			for (int j = 0; j < exec_stage->numFus(); j++) {
				ParExeStage* fu_stage = exec_stage->fu(j)->firstStage();
				
				// for each stage in the functional unit
				for (int k=0; k < fu_stage->numNodes(); k++) {
					ParExeInst* inst = fu_stage->node(k)->inst();

					// get cycle_time_info of the instruction
					r52f_time_t* inst_cycle_timing = getInstCycleTiming(inst->inst());

					ParExeNode* requiring_node = inst->firstFUNode();				
					if (inst->inst()->kind() & Inst::IS_SHIFT)
						requiring_node = findIssStage(inst);

					// for each instruction producing a used data
					for (ParExeInst::ProducingInstIterator prod(inst); prod(); prod ++) {
						ParExeNode* producing_node = prod->lastFUNode();
						if (prod->inst()->getKind().oneOf(Inst::IS_MUL | Inst::IS_MEM))
							producing_node = findWrSstage(*prod);
						// create the edge
						if (producing_node != nullptr && requiring_node != nullptr) 
							new ParExeEdge(producing_node, requiring_node, ParExeEdge::SOLID, 1, data_dep);
					}
				}
			}
		}
		
		/*
			Write to the log file, some info about the instructions whose
			cycle timing info has not been found.
		*/
		void dumpUnknowInst() {
			if (_out == nullptr)
				return;
			for (InstIterator inst(this); inst(); inst++) {
				if (!getInstCycleTiming(inst->inst())->unknown)
					continue;
				
				auto addr = inst->inst()->address();
				if (_unknown_inst_address->contains(addr))
					continue;
				_unknown_inst_address->add(addr);
				*_out << addr << "; " << inst->inst() << endl;
			}
		}
		
		void removeSimdAndFloatWrLatencies() {
			for (InstIterator inst(this); inst(); inst++) {
				if (inst->lastFUNode()->name().startsWith("FPU_SIMD"))
					findWrSstage(*inst)->setLatency(0);
			}
		}


		void build(void) override {			
			// Look for FUs
			for (ParExePipeline::StageIterator pipeline_stage(_microprocessor->pipeline()); pipeline_stage(); pipeline_stage++) {
				if (pipeline_stage->name() == "PreFetch") {
					stage[FE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "Decode") {
					stage[DE] = *pipeline_stage;
				} else if (pipeline_stage->name() == "Issue") {
                    stage[ISS] = *pipeline_stage;
                } else if (pipeline_stage->name() == "EXE") {
					// _microprocessor->setExecStage(*pipeline_stage);
					stage[EXE] = *pipeline_stage;
					for (int i = 0; i < pipeline_stage->numFus(); i++) {
						ParExePipeline* fu = pipeline_stage->fu(i);
						if (fu->firstStage()->name().startsWith("FPU_SIMD")) {
							exec_fpu_simd = fu;
						} else if (fu->firstStage()->name().startsWith("INTEGER")) {
							exec_int = fu;
						} else
							ASSERTP(false, fu->firstStage()->name() << ": Unknown functionnal-unit description found.");
					}
				} else if (pipeline_stage->name() == "Write") {
					stage[WR] = *pipeline_stage;
				} 

			}
			ASSERTP(stage[FE], "No 'Prefetch' stage found");
			ASSERTP(stage[DE], "No 'Decode' stage found");
            ASSERTP(stage[ISS], "No 'Issue' stage found");
			ASSERTP(stage[EXE], "No 'EXE' stage found");
			ASSERTP(stage[WR], "No 'Write back' stage found");
			ASSERTP(exec_int, "No 'INTEGER' fu found");
			ASSERTP(exec_fpu_simd, "No 'FPU/SIMD' fu found");
			

			// Build the execution graph 
			createSequenceResources();
			createNodes();
			addEdgesForPipelineOrder();
			addEdgesForFetch();
			addEdgesForProgramOrder();
			addEdgesForMemoryOrder();
			addEdgesForDataDependencies();
			removeSimdAndFloatWrLatencies();
			dumpUnknowInst();
		}

		
	private:
		otawa::arm::Info* info;
		const hard::Memory* mem;
		ParExeStage* stage[CNT];
		ParExePipeline *exec_fpu_simd, *exec_int;
		FileOutput* _out = nullptr;
		elm::Vector<Address>* _unknown_inst_address = nullptr;
		
		
		r52f_time_t* getInstCycleTiming(Inst* inst) {
			void* inst_info = info->decode(inst);
			r52f_time_t* inst_cycle_timing = ngUltraR52F(inst_info);
			info->free(inst_info);
			return inst_cycle_timing;
		}

		
		/*
			Find the ISS stage of an instruction.
			    inst: Concerned instruction.
		*/
		ParExeNode* findIssStage(ParExeInst* inst) {
			for (ParExeInst::NodeIterator node(inst); node(); node++) {
					if (node->stage() == stage[ISS])
						return *node;
			}
			return nullptr;
		}

		/*
			Find the WB stage of an instruction.
			    inst: Concerned instruction.
		*/
		ParExeNode* findWrSstage(ParExeInst* inst) {
			for (ParExeInst::NodeIterator node(inst); node(); node++) {
					if (node->stage() == stage[WR])
						return *node;
			}
			return nullptr;
		}

	};


	class BBTimerNGUltraR52: public etime::EdgeTimeBuilder {
	public:
		static p::declare reg;
		BBTimerNGUltraR52(void): etime::EdgeTimeBuilder(reg) { }

	protected:
		virtual void configure(const PropList& props) {
			etime::EdgeTimeBuilder::configure(props);
			write_log = WRITE_LOG(props);
			_props = props;
		}
		void setup(WorkSpace* ws) override {
			etime::EdgeTimeBuilder::setup(ws);
			const hard::CacheConfiguration* cache_config = hard::CACHE_CONFIGURATION_FEATURE.get(ws);
			if (!cache_config)
				throw ProcessorException(*this, "no cache");

			if (!cache_config->hasDataCache())
				throw ProcessorException(*this, "no data cache");

			if (!cache_config->hasInstCache())
				throw ProcessorException(*this, "no instruction cache");

			if (cache_config->isUnified())
				throw ProcessorException(*this, "unified L1 cache not supported");
			if (write_log) {
				sys::Path log_file_path = sys::Path(ws->process()->program()->name() + ".log");
				bool write_header = (log_file_path.exists()) ? false : true;
				log_stream = new FileOutput(log_file_path, true);
				if (write_header)
					*log_stream << "########################################################" << endl
								<< "# Static analysis on " << ws->process()->program()->name() << endl
								<< "# Overestimated instructions" << endl
								<< "# Address (hex); Instruction" << endl
								<< "########################################################" << endl;
				else
					*log_stream << endl; // sep
				
				unknown_inst_address = new elm::Vector<Address>();
			}
		}

		etime::EdgeTimeGraph* make(ParExeSequence* seq) override {
			R52FExeGraph* graph = new R52FExeGraph(workspace(), _microprocessor, ressources(), seq, _props, log_stream, unknown_inst_address);
			graph->build();
			return graph;
		}


		virtual void clean(ParExeGraph* graph) {
			log_stream->flush();
			delete graph;
		}
	private:
		PropList _props;
		FileOutput* log_stream = nullptr;
		bool write_log = 0;
		elm::Vector<Address>* unknown_inst_address = nullptr;
	};

	

	p::declare BBTimerNGUltraR52::reg = p::init("otawa::ultra::BBTimerNGUltraR52", Version(1, 0, 0))
										.extend<etime::EdgeTimeBuilder>()
										.require(otawa::hard::CACHE_CONFIGURATION_FEATURE)
										.maker<BBTimerNGUltraR52>();
	

} // namespace ultra
} // namespace otawa
